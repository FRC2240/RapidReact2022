#include "Shooter.h"

#include <iostream>

#include <fmt/core.h>

#include <math.h>

#include <frc/smartdashboard/SmartDashboard.h>


Shooter::Shooter(frc::DifferentialDrive* d, frc::XboxController* s, Take* t)
    : m_drive(d),
      m_stick(s),
      m_take(t)
{
  InitializeDashboard();
  InitializePIDControllers();
}

// Weak l'il spit
//Default value not working, investigate later
void Shooter::Spit(double vel = 0.1) {
  m_shootingMotorAlpha.Set(vel);
  m_shootingMotorBeta.Set(vel);
}

bool Shooter::LimelightTracking()
{


  double ty = m_table->GetNumber("ty", 0.0);
  double tx = m_table->GetNumber("tx", 0.0);
  double ta = m_table->GetNumber("ta", 0.0);


  std::cout << " TX: " << tx << "  TY: " << ty << "   TA: " << ta << "\n";

  /*
  //  nt::NetworkTableEntry txEntry;
  //  nt::NetworkTableEntry tyEntry;
  //  nt::NetworkTableEntry taEntry;

  double ty = m_table->GetNumber("ty", 0.0);
  double tx = m_table->GetNumber("tx", 0.0);
  double ta = m_table->GetNumber("ta", 0.0);


  //txEntry = m_table->GetEntry("tx");
  //tyEntry = m_table->GetEntry("ty");
  //  taEntry = m_table->GetEntry("ta");
  /*
  tx = txEntry.GetDouble(0.0);
  ty = tyEntry.GetDouble(0.0);
  ta = taEntry.GetDouble(0.0);
  */
  /*
  // Dummy values
  if (ta <= taHighBound &&
      ta >= taLowBound &&
      tx <= txHighBound &&
      tx >= txLowBound &&
      ty <= tyHighBound &&
      ty >= tyLowBound)
  {
    return true;
  }
  else
  {
    return false;
  }
  */
}

double Shooter::CalculateRPM(double d)
{
  return 0.0; // TODO: Add equation
}

//Shooter::Arm was removed as it was a relic of an old shooter

void Shooter::Fire() {
  if (m_overridenRPM != 0) {
    m_shootingMotorAlpha.Set(ControlMode::Velocity, m_overridenRPM*(2048.0/600.0));
    m_shootingMotorBeta.Set(ControlMode::Velocity, m_overridenRPM*(2048.0/600.0));
  }
  if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed)
  {
    if (m_take->BallColorRoom() == m_take->redBall)
    {
      wrongBall = false;
    }
    if (m_take->BallColorRoom() == m_take->blueBall)
    {
      wrongBall = true;
    }
    if (m_take->BallColorRoom() == m_take->errorBall)
    {
      std::cout << "[WARN]: Color Sensor issue \n";
    }
  }

  if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue)
  {
    if (m_take->BallColorRoom() == m_take->redBall)
    {
      wrongBall = true;
    }
    if (m_take->BallColorRoom() == m_take->blueBall)
    {
      wrongBall = false;
    }
    if (m_take->BallColorRoom() == m_take->errorBall)
    {
      std::cout << "[WARN]: Color Sensor issue \n";
    }
  }

  if (wrongBall)
  {
    m_stick->SetRumble(frc::GenericHID::RumbleType::kLeftRumble, 1.0);
    m_stick->SetRumble(frc::GenericHID::RumbleType::kRightRumble, 1.0);
  }
  if (!wrongBall)
  {
    m_stick->SetRumble(frc::GenericHID::RumbleType::kLeftRumble, 0.0);
    m_stick->SetRumble(frc::GenericHID::RumbleType::kRightRumble, 0.0);
  }

  nt::NetworkTableEntry txEntry;
  nt::NetworkTableEntry tyEntry;
  nt::NetworkTableEntry taEntry;

  double /*ta,*/ tx, ty;

  txEntry = m_table->GetEntry("tx");
  tyEntry = m_table->GetEntry("ty");
  taEntry = m_table->GetEntry("ta");

  tx = txEntry.GetDouble(0.0);
  ty = tyEntry.GetDouble(0.0);
  // ta = taEntry.GetDouble(0.0);

  ty = ty * -1;

  if (shootMan)
  {
    if (LimelightTracking() == true)
    {
      // Code stolen. Procedure is to map ty to theta, subtract a value I forget and then you get your angle. Solve from there.

      double distance = ((heightOfTarget - heightLimelight) / tan((constantLimelightAngle + ty) * (3.141592653 / 180)));

      double rpm = CalculateRPM(distance);
      /*
      m_shooterAlphaPIDController.SetReference(rpm, rev::CANSparkMax::ControlType::kVelocity);
      m_shooterBetaPIDController.SetReference(rpm, rev::CANSparkMax::ControlType::kVelocity);
      */
     m_shootingMotorAlpha.Set(ControlMode::Velocity, rpm*(2048.0/600.0));
     m_shootingMotorBeta.Set(ControlMode::Velocity, rpm*(2048.0/600.0));
    }
    else
    {
      if (tx < 0)
      {
        m_drive->ArcadeDrive(0, 0.5);
      }
      if (tx > 0)
      {
        m_drive->ArcadeDrive(0, -0.5);
      }
    }
  }

  if (!shootMan)
  {
    // Code stolen. Procedure is to map ty to theta, subtract a value I forget and then you get your angle. Solve from there.
    double distance = Shooter::LimelightDistance();

    double rpm = CalculateRPM(distance);
    /*
    m_shooterAlphaPIDController.SetReference(rpm, rev::CANSparkMax::ControlType::kVelocity);
    m_shooterBetaPIDController.SetReference(rpm, rev::CANSparkMax::ControlType::kVelocity);
    */
   m_shootingMotorAlpha.Set(ControlMode::Velocity, rpm*(2048.0/600.0));
   m_shootingMotorBeta.Set(ControlMode::Velocity, rpm*(2048.0/600.0));

  }
}

double Shooter::LimelightDistance(){

  //Yet another limelight initalization block
  nt::NetworkTableEntry txEntry;
  nt::NetworkTableEntry tyEntry;
  nt::NetworkTableEntry taEntry;

  double ta, tx, ty;

  txEntry = m_table->GetEntry("tx");
  tyEntry = m_table->GetEntry("ty");
  taEntry = m_table->GetEntry("ta");

  txEntry.SetDouble(tx);
  tyEntry.SetDouble(ty);
  taEntry.SetDouble(ta);

  std::cout << "TX: " << tx << " TY: " << ty << " TA: " << ta << "\n";

  //Map tyEntry (0-1) to Limelight FOV (0 - 49.7 deg)

  //How I got constants:
  //49.7 is the limelight fov
  // 36.711 is the angle of the limelight
  double theta = (49.7*ty)+36.711;

  // Then we trig

  //TODO: Better comments 
  double limelightDistance = (76.545/tan(theta));
  std::cout << "Limelight Distance" << limelightDistance << "\n";

  return limelightDistance;
}

void Shooter::InitializePIDControllers()
{
  ReadDashboard();
  /* Factory default hardware to prevent unexpected behavior */
    m_shootingMotorAlpha.ConfigFactoryDefault();
    m_shootingMotorBeta.ConfigFactoryDefault();

    /* Configure Sensor Source for Pirmary PID */
    m_shootingMotorAlpha.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 10);
    m_shootingMotorBeta.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 10);

    //_talon.SetSensorPhase(false);
    m_shootingMotorAlpha.SetInverted(TalonFXInvertType::CounterClockwise);
    m_shootingMotorBeta.SetInverted(TalonFXInvertType::CounterClockwise);

    /* Set relevant frame periods to be at least as fast as periodic rate */
    m_shootingMotorAlpha.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10, 10);
    m_shootingMotorAlpha.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, 10);

    m_shootingMotorBeta.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10, 10);
    m_shootingMotorBeta.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, 10);

    /* Set the peak and nominal outputs */
    m_shootingMotorAlpha.ConfigNominalOutputForward(0, 10);
    m_shootingMotorAlpha.ConfigNominalOutputReverse(0, 10);
    m_shootingMotorAlpha.ConfigPeakOutputForward(m_shooterAlphaCoeff.kMaxOutput, 10);
    m_shootingMotorAlpha.ConfigPeakOutputReverse(m_shooterAlphaCoeff.kMinOutput, 10);

    m_shootingMotorBeta.ConfigNominalOutputForward(0, 10);
    m_shootingMotorBeta.ConfigNominalOutputReverse(0, 10);
    m_shootingMotorBeta.ConfigPeakOutputForward(m_shooterBetaCoeff.kMaxOutput, 10);
    m_shootingMotorBeta.ConfigPeakOutputReverse(m_shooterBetaCoeff.kMinOutput, 10);

    /* Set Motion Magic gains in slot0 - see documentation */
    m_shootingMotorAlpha.SelectProfileSlot(0, 0);
    m_shootingMotorAlpha.Config_kF(0, m_shooterAlphaCoeff.kFF, 10);
    m_shootingMotorAlpha.Config_kP(0, m_shooterAlphaCoeff.kP, 10);
    m_shootingMotorAlpha.Config_kI(0, m_shooterAlphaCoeff.kI, 10);
    m_shootingMotorAlpha.Config_kD(0, m_shooterAlphaCoeff.kD, 10);

    m_shootingMotorBeta.SelectProfileSlot(0, 0);
    m_shootingMotorBeta.Config_kF(0, m_shooterBetaCoeff.kFF, 10);
    m_shootingMotorBeta.Config_kP(0, m_shooterBetaCoeff.kP, 10);
    m_shootingMotorBeta.Config_kI(0, m_shooterBetaCoeff.kI, 10);
    m_shootingMotorBeta.Config_kD(0, m_shooterBetaCoeff.kD, 10);

    /* Set acceleration and vcruise velocity - see documentation */
    m_shootingMotorAlpha.ConfigMotionCruiseVelocity(1500, 10);
    m_shootingMotorAlpha.ConfigMotionAcceleration(1500, 10);

    m_shootingMotorBeta.ConfigMotionCruiseVelocity(1500, 10);
    m_shootingMotorBeta.ConfigMotionAcceleration(1500, 10);

    /* Zero the sensor */
    m_shootingMotorBeta.SetSelectedSensorPosition(0, 0, 10);
    m_shootingMotorBeta.SetSelectedSensorPosition(0, 0, 10);
}

void Shooter::InitializeDashboard()
{
  // Belle bell

  frc::SmartDashboard::PutNumber("Shooter Alpha RPM",     m_shootingMotorAlpha.GetSelectedSensorVelocity());
  frc::SmartDashboard::PutNumber("Shooter Beta RPM",     m_shootingMotorBeta.GetSelectedSensorVelocity());

  if (m_overridenRPM != 0) { frc::SmartDashboard::PutNumber("Overridden RPM", m_overridenRPM);}

  frc::SmartDashboard::PutNumber("Desired RPM", CalculateRPM(LimelightDistance()));
  frc::SmartDashboard::PutNumber("Reported Distance", LimelightDistance());

  frc::SmartDashboard::PutNumber("Alpha P Gain", m_shooterAlphaCoeff.kP);
  frc::SmartDashboard::PutNumber("Alpha I Gain", m_shooterAlphaCoeff.kI);
  frc::SmartDashboard::PutNumber("Alpha D Gain", m_shooterAlphaCoeff.kD);
  frc::SmartDashboard::PutNumber("Alpha FF Gain", m_shooterAlphaCoeff.kFF);
  frc::SmartDashboard::PutNumber("Alpha Max Output", m_shooterAlphaCoeff.kMaxOutput);
  frc::SmartDashboard::PutNumber("Alpha Min Output", m_shooterAlphaCoeff.kMinOutput);

  frc::SmartDashboard::PutNumber("Beta P Gain", m_shooterBetaCoeff.kP);
  frc::SmartDashboard::PutNumber("Beta I Gain", m_shooterBetaCoeff.kI);
  frc::SmartDashboard::PutNumber("Beta D Gain", m_shooterBetaCoeff.kD);
  frc::SmartDashboard::PutNumber("Beta FF Gain", m_shooterBetaCoeff.kFF);
  frc::SmartDashboard::PutNumber("Beta Max Output", m_shooterBetaCoeff.kMaxOutput);
  frc::SmartDashboard::PutNumber("Beta Min Output", m_shooterBetaCoeff.kMinOutput);
}

void Shooter::ReadDashboard()
{

  m_overridenRPM = frc::SmartDashboard::GetNumber("Overridden RPM", 0.0);

  m_shooterAlphaCoeff.kP   = frc::SmartDashboard::GetNumber("Alpha P Gain", 0.0);
  m_shooterAlphaCoeff.kI   = frc::SmartDashboard::GetNumber("Alpha I Gain", 0.0);
  m_shooterAlphaCoeff.kD   = frc::SmartDashboard::GetNumber("Alpha D Gain", 0.0);
  m_shooterAlphaCoeff.kFF  = frc::SmartDashboard::GetNumber("Alpha FF Gain", 0.0);
  m_shooterAlphaCoeff.kMinOutput = frc::SmartDashboard::GetNumber("Alpha Min Output", 0.0);
  m_shooterAlphaCoeff.kMaxOutput = frc::SmartDashboard::GetNumber("Alpha Max Output", 0.0);

  m_shooterBetaCoeff.kP   = frc::SmartDashboard::GetNumber("Beta P Gain", 0.0);
  m_shooterBetaCoeff.kI   = frc::SmartDashboard::GetNumber("Beta I Gain", 0.0);
  m_shooterBetaCoeff.kD   = frc::SmartDashboard::GetNumber("Beta D Gain", 0.0);
  m_shooterBetaCoeff.kFF  = frc::SmartDashboard::GetNumber("Beta FF Gain", 0.0);
  m_shooterBetaCoeff.kMinOutput = frc::SmartDashboard::GetNumber("Beta Min Output", 0.0);
  m_shooterBetaCoeff.kMaxOutput = frc::SmartDashboard::GetNumber("Beta Max Output", 0.0);

}
