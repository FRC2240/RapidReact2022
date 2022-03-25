#include "Take.h"
// #include "log.h"
#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>



#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

Take::Take() {
  //TakeDashInit();
  TakePIDInit();

  m_waitingRoomPIDController.SetP(m_waitingRoomCoeff.kP);
  m_waitingRoomPIDController.SetI(m_waitingRoomCoeff.kI);
  m_waitingRoomPIDController.SetD(m_waitingRoomCoeff.kD);
  m_waitingRoomPIDController.SetIZone(m_waitingRoomCoeff.kIz);
  m_waitingRoomPIDController.SetFF(m_waitingRoomCoeff.kFF);
  m_waitingRoomPIDController.SetOutputRange(m_waitingRoomCoeff.kMinOutput, m_waitingRoomCoeff.kMaxOutput);

  // Setup external encoder
  m_rotateIntakeEncoder.SetInverted(false);
  m_rotateIntakeEncoder.SetPosition(0.0);
  m_rotateIntakePIDController.SetFeedbackDevice(m_rotateIntakeEncoder);
}

void Take::Run(bool toggle, bool shooting, bool autonomous, frc::DriverStation::Alliance alliance) {
  // Events that will affect state:
  // - Driver input
  // - Uptake/Waiting Room become full
  // - Wrong color detected
  // - Eject complete
  auto currentState = m_state;


  if (toggle || currentState != Off) {
    ReadSensors(toggle);
  }

  // Driver input?
  if (toggle && currentState == Intaking) {
    m_state = Off;
  }

  // Driver input?
  if (toggle && currentState == Off) {
    m_state = Intaking;
  }

  // Full?
  if ((m_waitingRoomState != nullBall) && (m_uptakeState != nullBall)) {
    m_state = Off;
  }

  // Wrong color in uptake?
  if (!autonomous) {
    if (WrongColor(m_uptakeState, alliance)) {
      m_state = Ejecting;
      m_ejectTimer = 40;
    }
  }

  // Timer complete?
  if (m_ejectTimer != 0) {
    --m_ejectTimer;

    if (m_ejectTimer == 0) {
      m_state = Intaking;
    }
  }

  // Shooting? Disable intake
  if (shooting) {
    m_state = Off;
    m_ejectTimer = 0;
  }

  // Intake ON or OFF (manual toggle or auto-stop because we're full)
  if (m_state != currentState)
  {
    switch (m_state) {
      case Ejecting:
        m_uptakeMotor.Set(1.0);
        m_spinIntakeMotor.Set(1.0);
        break;
      case Intaking:
        DeployIntake();
        m_spinIntakeMotor.Set(-1.0);
        m_uptakeMotor.Set(-0.3);
        break;
      case Off:
      default:
        m_uptakeMotor.Set(0.0);
        m_spinIntakeMotor.Set(0.0);
        ReturnIntake();
    }
  }
}

void Take::ReadSensors(bool toggle) {
  static int count = 0;

  ++count;

  // Only read the I2C data once per 10 loops
  // However, force a read if it's a toggle to update the ball states
  if ((count < 10) && !toggle) {
    return;
  }

  // Read data
  auto uptake  = m_uptakeSensor.GetColor();
  auto waiting = m_waitingRoomSensor.GetColor();

  // Determine colors
  m_uptakeState      = Color(uptake);
  m_waitingRoomState = Color(waiting);


  //Shuffleboard
  if (m_uptakeState == blueBall) {
    m_uptakeBallBlueBoard.SetBoolean(true);
      }
  else{
    m_uptakeBallBlueBoard.SetBoolean(false);
  }
  //---
  if (m_uptakeState == redBall) {
    m_uptakeBallRedBoard.SetBoolean(true);
  }
  else {
    m_uptakeBallRedBoard.SetBoolean(false);
  }
  //---
  if (m_waitingRoomState == redBall) {
    m_roomBallRedBoard.SetBoolean(true);
      }
  else {
    m_roomBallRedBoard.SetBoolean(false);
      }
  //---
  if (m_waitingRoomState == blueBall) {
    m_roomBallBlueBoard.SetBoolean(true);
  }
  else {
    m_roomBallBlueBoard.SetBoolean(false);
  }

  count = 0;
}

void Take::Feed(double speedWR, double speedUptake) {
  m_uptakeMotor.Set(-speedUptake);
  m_waitingRoomMotor.Set(speedWR);
}

void Take::UptakeStart(double speed)
{
  m_spinIntakeMotor.Set(speed);
  m_uptakeMotor.Set(speed);
}

void Take::UptakeStop()
{
  m_spinIntakeMotor.Set(0);
  m_uptakeMotor.Set(0);
}

void Take::DeployIntake()
{
  m_rotateIntakePIDController.SetReference(0.25, rev::CANSparkMax::ControlType::kSmartMotion);
}

void Take::ReturnIntake()
{
  m_rotateIntakePIDController.SetReference(0.0, rev::CANSparkMax::ControlType::kSmartMotion);
}

void Take::AutoRunIntake(double speed){
  m_spinIntakeMotor.Set(speed);
  m_uptakeMotor.Set(speed);
}

void Take::AutoStopIntake(){
  m_spinIntakeMotor.Set(0);
  m_uptakeMotor.Set(0);
}

// Measurements from REV Color Sensors:
// RED  BALL: R (0.40 - 0.54),  G (0.34 - 0.40), B (0.12 - 0.20)
// BLUE BALL: R (0.14 - 0.20),  G (0.39 - 0.44), B (0.36 - 0.47)
//   NO BALL: R (0.25), G (0.47), B (0.28)
Take::BallColor Take::Color(frc::Color color) {
  if ((color.red > 0.37) && (color.blue < 0.23)) {
    return Take::redBall;
  }

  if ((color.red < 0.23) && (color.blue > 0.33)) {
    return Take::blueBall;
  }

  return Take::nullBall;
}

bool Take::WrongColor(BallColor ball, frc::DriverStation::Alliance alliance)
{
  if (ball == blueBall && alliance == frc::DriverStation::Alliance::kRed) { return true; }
  if (ball == redBall && alliance == frc::DriverStation::Alliance::kBlue) { return true; }

  return false;
}

// To be replaced with constructor
void Take::TakePIDInit()
{
  m_rotateIntakePIDController.SetP(m_rotateIntakeCoeff.kP);
  m_rotateIntakePIDController.SetI(m_rotateIntakeCoeff.kI);
  m_rotateIntakePIDController.SetD(m_rotateIntakeCoeff.kD);
  m_rotateIntakePIDController.SetIZone(m_rotateIntakeCoeff.kIz);
  m_rotateIntakePIDController.SetFF(m_rotateIntakeCoeff.kFF);
  m_rotateIntakePIDController.SetOutputRange(m_rotateIntakeCoeff.kMinOutput, m_rotateIntakeCoeff.kMaxOutput);

  m_rotateIntakePIDController.SetSmartMotionMaxVelocity(kMaxVel);
  m_rotateIntakePIDController.SetSmartMotionMinOutputVelocity(kMinVel);
  m_rotateIntakePIDController.SetSmartMotionMaxAccel(kMaxAcc);
  m_rotateIntakePIDController.SetSmartMotionAllowedClosedLoopError(kAllErr);

  m_uptakePIDController.SetP(m_uptakeCoeff.kP);
  m_uptakePIDController.SetI(m_uptakeCoeff.kI);
  m_uptakePIDController.SetD(m_uptakeCoeff.kD);
  m_uptakePIDController.SetIZone(m_uptakeCoeff.kIz);
  m_uptakePIDController.SetFF(m_uptakeCoeff.kFF);
  m_uptakePIDController.SetOutputRange(m_uptakeCoeff.kMinOutput, m_uptakeCoeff.kMaxOutput);

  m_waitingRoomPIDController.SetP(m_waitingRoomCoeff.kP);
  m_waitingRoomPIDController.SetI(m_waitingRoomCoeff.kI);
  m_waitingRoomPIDController.SetD(m_waitingRoomCoeff.kD);
  m_waitingRoomPIDController.SetIZone(m_waitingRoomCoeff.kIz);
  m_waitingRoomPIDController.SetFF(m_waitingRoomCoeff.kFF);
  m_waitingRoomPIDController.SetOutputRange(m_waitingRoomCoeff.kMinOutput, m_waitingRoomCoeff.kMaxOutput);
}

void Take::TakeDashInit()
{
  // Rotate intake
  frc::SmartDashboard::PutNumber("Rotate Intake P Gain", m_rotateIntakeCoeff.kP);
  frc::SmartDashboard::PutNumber("Rotate Intake I Gain", m_rotateIntakeCoeff.kI);
  frc::SmartDashboard::PutNumber("Rotate Intake D Gain", m_rotateIntakeCoeff.kD);
  frc::SmartDashboard::PutNumber("Rotate Intake Max Output", m_rotateIntakeCoeff.kMaxOutput);
  frc::SmartDashboard::PutNumber("Rotate Intake Min Output", m_rotateIntakeCoeff.kMinOutput);

  frc::SmartDashboard::PutNumber("Uptake P Gain", m_uptakeCoeff.kP);
  frc::SmartDashboard::PutNumber("Uptake I Gain", m_uptakeCoeff.kI);
  frc::SmartDashboard::PutNumber("Uptake D Gain", m_uptakeCoeff.kD);
  frc::SmartDashboard::PutNumber("Uptake Max Output", m_uptakeCoeff.kMaxOutput);
  frc::SmartDashboard::PutNumber("Uptake Min Output", m_uptakeCoeff.kMinOutput);

  frc::SmartDashboard::PutNumber("Waiting Room P Gain", m_waitingRoomCoeff.kP);
  frc::SmartDashboard::PutNumber("Waiting Room I Gain", m_waitingRoomCoeff.kI);
  frc::SmartDashboard::PutNumber("Waiting Room D Gain", m_waitingRoomCoeff.kD);
  frc::SmartDashboard::PutNumber("Waiting Room Max Output", m_waitingRoomCoeff.kMaxOutput);
  frc::SmartDashboard::PutNumber("Waiting Room Min Output", m_waitingRoomCoeff.kMinOutput);
}

void Take::TakeDashRead()
{
  double p, i, d, min, max;
  // rotate intake
  // read PID coefficients from SmartDashboard
  p = frc::SmartDashboard::GetNumber("Rotate Intake P Gain", 0);
  i = frc::SmartDashboard::GetNumber("Rotate Intake I Gain", 0);
  d = frc::SmartDashboard::GetNumber("Rotate Intake D Gain", 0);
  min = frc::SmartDashboard::GetNumber("Rotate Intake Min Output", 0);
  max = frc::SmartDashboard::GetNumber("Rotate Intake Max Output", 0);

  // If PID coefficients on SmartDashboard have changed, write new values to controller
  if ((p != m_rotateIntakeCoeff.kP))
  {
    m_rotateIntakePIDController.SetP(p);
    m_rotateIntakeCoeff.kP = p;
  }
  if ((i != m_rotateIntakeCoeff.kI))
  {
    m_rotateIntakePIDController.SetI(i);
    m_rotateIntakeCoeff.kI = i;
  }
  if ((d != m_rotateIntakeCoeff.kD))
  {
    m_rotateIntakePIDController.SetD(d);
    m_rotateIntakeCoeff.kD = d;
  }
  if ((max != m_rotateIntakeCoeff.kMaxOutput) || (min != m_rotateIntakeCoeff.kMinOutput))
  {
    m_rotateIntakePIDController.SetOutputRange(min, max);
    m_rotateIntakeCoeff.kMinOutput = min;
    m_rotateIntakeCoeff.kMaxOutput = max;
  }

  p = frc::SmartDashboard::GetNumber("Uptake P Gain", 0);
  i = frc::SmartDashboard::GetNumber("Uptake I Gain", 0);
  d = frc::SmartDashboard::GetNumber("Uptake D Gain", 0);
  min = frc::SmartDashboard::GetNumber("Uptake Min Output", 0);
  max = frc::SmartDashboard::GetNumber("Uptake Max Output", 0);

  if ((p != m_uptakeCoeff.kP))
  {
    m_uptakePIDController.SetP(p);
    m_uptakeCoeff.kP = p;
  }
  if ((i != m_uptakeCoeff.kI))
  {
    m_uptakePIDController.SetI(i);
    m_uptakeCoeff.kI = i;
  }
  if ((d != m_uptakeCoeff.kD))
  {
    m_uptakePIDController.SetD(d);
    m_uptakeCoeff.kD = d;
  }
  if ((max != m_uptakeCoeff.kMaxOutput) || (min != m_uptakeCoeff.kMinOutput))
  {
    m_uptakePIDController.SetOutputRange(min, max);
    m_uptakeCoeff.kMinOutput = min;
    m_uptakeCoeff.kMaxOutput = max;
  }

  p = frc::SmartDashboard::GetNumber("Waiting Room P Gain", 0);
  i = frc::SmartDashboard::GetNumber("Waiting Room I Gain", 0);
  d = frc::SmartDashboard::GetNumber("Waiting Room D Gain", 0);
  min = frc::SmartDashboard::GetNumber("Waiting Room Min Output", 0);
  max = frc::SmartDashboard::GetNumber("Waiting Room Max Output", 0);

  if ((p != m_waitingRoomCoeff.kP))
  {
    m_waitingRoomPIDController.SetP(p);
    m_waitingRoomCoeff.kP = p;
  }
  if ((i != m_waitingRoomCoeff.kI))
  {
    m_waitingRoomPIDController.SetI(i);
    m_waitingRoomCoeff.kI = i;
  }
  if ((d != m_waitingRoomCoeff.kD))
  {
    m_waitingRoomPIDController.SetD(d);
    m_waitingRoomCoeff.kD = d;
  }
  if ((max != m_waitingRoomCoeff.kMaxOutput) || (min != m_waitingRoomCoeff.kMinOutput))
  {
    m_waitingRoomPIDController.SetOutputRange(min, max);
    m_waitingRoomCoeff.kMinOutput = min;
    m_uptakeCoeff.kMaxOutput = max;
  }

  // FIXME
  frc::Color dashDetectedColorUptake; // = m_uptakeSensor.GetColor();

  frc::SmartDashboard::PutNumber("Red", dashDetectedColorUptake.red);
  frc::SmartDashboard::PutNumber("Green", dashDetectedColorUptake.green);
  frc::SmartDashboard::PutNumber("Blue", dashDetectedColorUptake.blue);

  // FIXME
  frc::Color dashDetectedColorRoom;
  frc::SmartDashboard::PutNumber("Red", dashDetectedColorRoom.red);
  frc::SmartDashboard::PutNumber("Green", dashDetectedColorRoom.green);
  frc::SmartDashboard::PutNumber("Blue", dashDetectedColorRoom.blue);
}

void Take::InitializeEncoders() {
  m_rotateIntakeEncoder.SetPosition(0.0);
}

void Take::TestDashInit() {
  // Rotate intake
  frc::SmartDashboard::PutNumber("Rotate Intake P Gain", m_rotateIntakeCoeff.kP);
  frc::SmartDashboard::PutNumber("Rotate Intake I Gain", m_rotateIntakeCoeff.kI);
  frc::SmartDashboard::PutNumber("Rotate Intake D Gain", m_rotateIntakeCoeff.kD);
  frc::SmartDashboard::PutNumber("Rotate Intake Max Output", m_rotateIntakeCoeff.kMaxOutput);
  frc::SmartDashboard::PutNumber("Rotate Intake Min Output", m_rotateIntakeCoeff.kMinOutput);

  frc::SmartDashboard::PutNumber("Rotation Position", m_rotationPosition); 

}

void Take::TestDashRead() {
  // rotate intake
  m_rotateIntakeCoeff.kP = frc::SmartDashboard::GetNumber("Rotate Intake P Gain", 0.0);
  m_rotateIntakeCoeff.kI = frc::SmartDashboard::GetNumber("Rotate Intake I Gain", 0.0);
  m_rotateIntakeCoeff.kD = frc::SmartDashboard::GetNumber("Rotate Intake D Gain", 0.0);
  m_rotateIntakeCoeff.kMinOutput = frc::SmartDashboard::GetNumber("Rotate Intake Min Output", 0.0);
  m_rotateIntakeCoeff.kMaxOutput = frc::SmartDashboard::GetNumber("Rotate Intake Max Output", 0.0);
  
  m_rotationPosition = frc::SmartDashboard::GetNumber("Rotation Position", 0.0); 
}

void Take::SetIntakePosition(double position){
  m_rotateIntakePIDController.SetReference(position, rev::CANSparkMax::ControlType::kPosition);
}

void Take::TestRotation() {
  std::cout << "Rotation Point: " << m_rotationPosition << "\n";
  SetIntakePosition(m_rotationPosition); 
}

void Take::ReadEncoders() {
  frc::SmartDashboard::PutNumber("Rotate Intake Position: ", m_rotateIntakeEncoder.GetPosition()); 
}
