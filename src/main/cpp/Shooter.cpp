#include <iostream>
#include <math.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "Shooter.h"

Shooter::Shooter(frc::DifferentialDrive *d, frc::XboxController *s, Take *t)
    : m_drive(d),
      m_stick(s),
      m_take(t)
{
  InitializeDashboard();
  InitializePIDControllers();

  // Follow the alpha
  m_shootingMotorBeta.Follow(m_shootingMotorAlpha);
  m_shootingMotorBeta.SetInverted(InvertType::OpposeMaster);
}

// Reset
void Shooter::Reset()
{
  m_table->PutNumber("ledMode", 1); // lights off
  m_shootingMotorAlpha.Set(0.0);    // motors off
  m_take->Feed(0.0);                // feed off
}

// Weak l'il spit
// Default value not working, investigate later
void Shooter::Spit(double vel = 0.1)
{
  m_shootingMotorAlpha.Set(vel);
}

bool Shooter::LimelightTracking()
{
  bool shoot = false;

  // Proportional Steering Constant:
  // If your robot doesn't turn fast enough toward the target, make this number bigger
  // If your robot oscillates (swings back and forth past the target) make this smaller
  const double STEER_K = 0.04;
  const double MAX_STEER = 0.5;

  double tx = m_table->GetNumber("tx", 0.0);
  double tv = m_table->GetNumber("tv", 0.0);

  //std::cout << "tx: " << tx << "; tv: " << tv << "\n";

  double limelightTurnCmd = 0.0;

  if (tv > 0.0)
  {
    std::cout << "Limelight Tracking, tx: " << tx << "\n";
    // Proportional steering
    limelightTurnCmd = (tx /*+ m_txOFFSET*/) * STEER_K;
    limelightTurnCmd = std::clamp(limelightTurnCmd, -MAX_STEER, MAX_STEER);
    if (fabs(tx) < 3.0)
    {
      std::cout << "Shoot true \n";
      shoot = true;
    }
  }

  //std::cout << "Trying to turn: " << limelightTurnCmd << "\n"; 
  // Turn the robot to the target
  m_drive->ArcadeDrive(0.0, limelightTurnCmd);
  return shoot;
}

double Shooter::CalculateRPM(double d)
{
  return 2000.0; // TODO: Add equation
}

void Shooter::Fire()
{
  m_table->PutNumber("ledMode", 3); // lights on

  // Is target locked?
  if (LimelightTracking())
  {
    // Calculate distance to target from Limelight data
    double ty = m_table->GetNumber("ty", 0.0);
    std::cout << "ty: " << ty <<"\n";
    //double distance = kRadiusOfTarget + ((kHeightOfTarget - kHeightLimelight) / tan((kLimelightAngle + ty) * (3.141592653 / 180)));

    double distance = -2.39 * ty + (0.139 * pow(ty, 2)) + 105; 

    double rpm = CalculateRPM(distance);
    std::cout << "distance: " << distance << "\n";

    // Override for test/calibration?
    if (fabs(m_overrideRPM) > 1.0)
    {
      rpm = m_overrideRPM;
      frc::SmartDashboard::PutNumber("Shooter RPM", m_shootingMotorAlpha.GetSelectedSensorVelocity()*(600.0/2048.0));
    }

    if ((distance < 250) && (distance > 70))
    {
      std::cout << "RPM: " << rpm << "\n"; 
      m_shootingMotorAlpha.Set(ControlMode::Velocity, rpm * (2048.0 / 600.0));
    }
    // std::cout << "distance = " << distance
    //           << " want = " << rpm << " got = " << m_shootingMotorAlpha.GetSelectedSensorVelocity() << std::endl;

    // Enable feed if we're at 98% of desired shooter speed
    if (fabs(m_shootingMotorAlpha.GetSelectedSensorVelocity()) > fabs(rpm * 0.97))
    {
      m_take->Feed(1.0);
    } else {
      m_take->Feed(0.0);
    }
  }
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
  // m_shootingMotorAlpha.SetInverted(TalonFXInvertType::CounterClockwise);
  // m_shootingMotorBeta.SetInverted(TalonFXInvertType::CounterClockwise);

  /* Set relevant frame periods to be at least as fast as periodic rate */
  m_shootingMotorAlpha.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10, 10);
  m_shootingMotorAlpha.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, 10);

  m_shootingMotorBeta.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10, 10);
  m_shootingMotorBeta.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, 10);

  /* Set the peak and nominal outputs */
  m_shootingMotorAlpha.ConfigNominalOutputForward(0, 10);
  m_shootingMotorAlpha.ConfigNominalOutputReverse(0, 10);
  m_shootingMotorAlpha.ConfigPeakOutputForward(m_shooterCoeff.kMaxOutput, 10);
  m_shootingMotorAlpha.ConfigPeakOutputReverse(m_shooterCoeff.kMinOutput, -10);

  m_shootingMotorAlpha.SelectProfileSlot(0, 0);
  m_shootingMotorAlpha.Config_kF(0, m_shooterCoeff.kFF, 10);
  m_shootingMotorAlpha.Config_kP(0, m_shooterCoeff.kP, 10);
  m_shootingMotorAlpha.Config_kI(0, m_shooterCoeff.kI, 10);
  m_shootingMotorAlpha.Config_kD(0, m_shooterCoeff.kD, 10);
}

void Shooter::InitializeDashboard()
{
  frc::SmartDashboard::PutNumber("Overridden RPM", m_overrideRPM);

  frc::SmartDashboard::PutNumber("Shooter P Gain", m_shooterCoeff.kP);
  frc::SmartDashboard::PutNumber("Shooter I Gain", m_shooterCoeff.kI);
  frc::SmartDashboard::PutNumber("Shooter D Gain", m_shooterCoeff.kD);
  frc::SmartDashboard::PutNumber("Shooter FF Gain", m_shooterCoeff.kFF);
  frc::SmartDashboard::PutNumber("Shooter Max Output", m_shooterCoeff.kMaxOutput);
  frc::SmartDashboard::PutNumber("Shooter Min Output", m_shooterCoeff.kMinOutput);
}

void Shooter::ReadDashboard()
{
  m_overrideRPM = frc::SmartDashboard::GetNumber("Overridden RPM", 0.0);

  m_shooterCoeff.kP = frc::SmartDashboard::GetNumber("Shooter P Gain", 0.0);
  m_shooterCoeff.kI = frc::SmartDashboard::GetNumber("Shooter I Gain", 0.0);
  m_shooterCoeff.kD = frc::SmartDashboard::GetNumber("Shooter D Gain", 0.0);
  m_shooterCoeff.kFF = frc::SmartDashboard::GetNumber("Shooter FF Gain", 0.0);
  m_shooterCoeff.kMinOutput = frc::SmartDashboard::GetNumber("Shooter Min Output", 0.0);
  m_shooterCoeff.kMaxOutput = frc::SmartDashboard::GetNumber("Shooter Max Output", 0.0);
}
