#include "Take.h"
#include "log.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>


void Take::Run(bool toggle, frc::DriverStation::Alliance alliance)
{
  // Events that will affect state:
  // - Driver input
  // - Uptake/Waiting Room become full
  // - Wrong color detected
  // - Eject complete
  auto currentState = m_state;

  if (toggle || currentState != Off) {
    ReadSensors();
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
  if (WrongColor(m_uptakeState, alliance)) {
    m_state = Ejecting;
    m_ejectTimer = 40;
  }

  // Timer complete?
  if (m_ejectTimer != 0) {
    --m_ejectTimer;

    if (m_ejectTimer == 0) {
      m_state = Intaking;
    }
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
        m_uptakeMotor.Set(-0.4);
        break;
      case Off:
      default:
        m_uptakeMotor.Set(0.0);
        m_spinIntakeMotor.Set(0.0);
        ReturnIntake();
    }
  }
}

void Take::ReadSensors() {
  static int count = 0;

  ++count;

  // Only read the I2C data once per 10 loops
  if (count < 10) {
    return;
  }

  // Read data
  auto uptake  = m_uptakeSensor.GetColor();
  auto waiting = m_waitingRoomSensor.GetColor();

  // Determine colors
  m_uptakeState      = Color(uptake);
  m_waitingRoomState = Color(waiting);

  //std::cout << "Uptake: " << m_uptakeState << "Wa: " << m_waitingRoomState << std::endl;

  count = 0;
}

void Take::Feed(double speed) {
  m_uptakeMotor.Set(-speed);
  m_waitingRoomMotor.Set(speed);
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
  m_rotateIntakePIDController.SetReference(10.17, rev::CANSparkMax::ControlType::kSmartMotion);
}

void Take::ReturnIntake()
{
  m_rotateIntakePIDController.SetReference(0.0, rev::CANSparkMax::ControlType::kSmartMotion);
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
  std::cout << "Read Dashboard rotate intake p gain: " << p << "\n";
  i = frc::SmartDashboard::GetNumber("Rotate Intake I Gain", 0);
  std::cout << "Read Dashboard rotate intake i gain: " << i << "\n";
  d = frc::SmartDashboard::GetNumber("Rotate Intake D Gain", 0);
  std::cout << "Read Dashboard rotate intake d gain: " << d << "\n";
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
  std::cout << "Read Dashboard uptake p gain: " << p << "\n";
  i = frc::SmartDashboard::GetNumber("Uptake I Gain", 0);
  std::cout << "Read Dashboard uptake i gain: " << i << "\n";
  d = frc::SmartDashboard::GetNumber("Uptake D Gain", 0);
  std::cout << "Read Dashboard uptake d gain: " << d << "\n";
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
  std::cout << "Read Dashboard waiting room p gain: " << p << "\n";
  i = frc::SmartDashboard::GetNumber("Waiting Room I Gain", 0);
  std::cout << "Read Dashboard waiting room i gain: " << i << "\n";
  d = frc::SmartDashboard::GetNumber("Waiting Room D Gain", 0);
  std::cout << "Read Dashboard waiting room d gain: " << d << "\n";
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
  //    double dashUptakeIR; // = m_uptakeSensor.GetIR(); //unused

  frc::SmartDashboard::PutNumber("Red", dashDetectedColorUptake.red);
  frc::SmartDashboard::PutNumber("Green", dashDetectedColorUptake.green);
  frc::SmartDashboard::PutNumber("Blue", dashDetectedColorUptake.blue);
  //  frc::SmartDashboard::PutNumber("IR", dashUptakeIR); // UNUSED

  // FIXME
  frc::Color dashDetectedColorRoom; // = m_waitingRoomSensor.GetColor();
  //  double dashRoomIR; // = m_waitingRoomSensor.GetIR(); // Unused

  frc::SmartDashboard::PutNumber("Red", dashDetectedColorRoom.red);
  frc::SmartDashboard::PutNumber("Green", dashDetectedColorRoom.green);
  frc::SmartDashboard::PutNumber("Blue", dashDetectedColorRoom.blue);
  //  frc::SmartDashboard::PutNumber("IR", dashRoomIR); //Unsued
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

  frc::SmartDashboard::PutNumber("Intake Rotation Encoder: ", m_rotateIntakeEncoder.GetPosition());
}

void Take::TestDashRead() {
  double p, i, d, min, max;
  // rotate intake
  p = frc::SmartDashboard::GetNumber("Rotate Intake P Gain", 0.0);
  i = frc::SmartDashboard::GetNumber("Rotate Intake I Gain", 0.0);
  d = frc::SmartDashboard::GetNumber("Rotate Intake D Gain", 0.0);
  min = frc::SmartDashboard::GetNumber("Rotate Intake Min Output", 0.0);
  max = frc::SmartDashboard::GetNumber("Rotate Intake Max Output", 0.0);
}
