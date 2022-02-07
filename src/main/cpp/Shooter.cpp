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

bool Shooter::LimelightTracking()
{
  nt::NetworkTableEntry txEntry;
  nt::NetworkTableEntry tyEntry;
  nt::NetworkTableEntry taEntry;

  double ta, tx, ty;

  auto inst = nt::NetworkTableInstance::GetDefault();
  auto table = inst.GetTable("limelight-bepis");
  txEntry = table->GetEntry("tx");
  tyEntry = table->GetEntry("ty");
  taEntry = table->GetEntry("ta");

  tx = txEntry.GetDouble(0.0);
  ty = tyEntry.GetDouble(0.0);
  ta = taEntry.GetDouble(0.0);

  std::cout << tx << ty << ta << "\n";

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
}

double Shooter::CalculateRPM(double d)
{
  return 0.0; // TODO: Add equation
}

void Shooter::Arm() {}

void Shooter::Fire()
{
  if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed)
  {
    if (m_take->BallColorRoom() == 'r')
    {
      wrongBall = false;
    }
    if (m_take->BallColorRoom() == 'b')
    {
      wrongBall = true;
    }
    if (m_take->BallColorRoom() == 'E')
    {
      std::cout << "[WARN]: Color Sensor issue \n";
    }
  }

  if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue)
  {
    if (m_take->BallColorRoom() == 'r')
    {
      wrongBall = true;
    }
    if (m_take->BallColorRoom() == 'b')
    {
      wrongBall = false;
    }
    if (m_take->BallColorRoom() == 'E')
    {
      std::cout << "[WARN]: Color Sensor issue \n";
    }
  }

  if (wrongBall)
  {
    //  sosTimer.Start()
    // Make an SOS
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

  auto inst = nt::NetworkTableInstance::GetDefault();
  auto table = inst.GetTable("limelight-bepis");
  txEntry = table->GetEntry("tx");
  tyEntry = table->GetEntry("ty");
  taEntry = table->GetEntry("ta");

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
      m_shooterAlphaPIDController.SetReference(rpm, rev::CANSparkMax::ControlType::kVelocity);
      m_shooterBetaPIDController.SetReference(rpm, rev::CANSparkMax::ControlType::kVelocity);
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
    double distance = ((heightOfTarget - heightLimelight) / tan((constantLimelightAngle + ty) * (3.141592653 / 180)));

    double rpm = CalculateRPM(distance);
    m_shooterAlphaPIDController.SetReference(rpm, rev::CANSparkMax::ControlType::kVelocity);
    m_shooterBetaPIDController.SetReference(rpm, rev::CANSparkMax::ControlType::kVelocity);
  }
}

void Shooter::InitializePIDControllers()
{
  ReadDashboard();

  m_shooterAlphaPIDController.SetP(m_shooterAlphaCoeff.kP);
  m_shooterAlphaPIDController.SetI(m_shooterAlphaCoeff.kI);
  m_shooterAlphaPIDController.SetD(m_shooterAlphaCoeff.kD);
  m_shooterAlphaPIDController.SetIZone(m_shooterAlphaCoeff.kIz);
  m_shooterAlphaPIDController.SetFF(m_shooterAlphaCoeff.kFF);
  m_shooterAlphaPIDController.SetOutputRange(m_shooterAlphaCoeff.kMinOutput, m_shooterAlphaCoeff.kMaxOutput);

  m_shooterBetaPIDController.SetP(m_shooterBetaCoeff.kP);
  m_shooterBetaPIDController.SetI(m_shooterBetaCoeff.kI);
  m_shooterBetaPIDController.SetD(m_shooterBetaCoeff.kD);
  m_shooterBetaPIDController.SetIZone(m_shooterBetaCoeff.kIz);
  m_shooterBetaPIDController.SetFF(m_shooterBetaCoeff.kFF);
  m_shooterBetaPIDController.SetOutputRange(m_shooterBetaCoeff.kMinOutput, m_shooterBetaCoeff.kMaxOutput);
}

void Shooter::InitializeDashboard()
{
  frc::SmartDashboard::PutNumber("Alpha Motor P Gain", m_shooterAlphaCoeff.kP);
  frc::SmartDashboard::PutNumber("Alpha Motor I Gain", m_shooterAlphaCoeff.kI);
  frc::SmartDashboard::PutNumber("Alpha Motor D Gain", m_shooterAlphaCoeff.kD);
  frc::SmartDashboard::PutNumber("Alpha Motor Max Output", m_shooterAlphaCoeff.kMaxOutput);
  frc::SmartDashboard::PutNumber("Alpha Motor Min Output", m_shooterAlphaCoeff.kMinOutput);

  frc::SmartDashboard::PutNumber("Beta Motor P Gain", m_shooterBetaCoeff.kP);
  frc::SmartDashboard::PutNumber("Beta Motor I Gain", m_shooterBetaCoeff.kI);
  frc::SmartDashboard::PutNumber("Beta Motor D Gain", m_shooterBetaCoeff.kD);
  frc::SmartDashboard::PutNumber("Beta Motor Max Output", m_shooterBetaCoeff.kMaxOutput);
  frc::SmartDashboard::PutNumber("Beta Motor Min Output", m_shooterBetaCoeff.kMinOutput);
}

void Shooter::ReadDashboard()
{
  double p, i, d, min, max;

  p = frc::SmartDashboard::GetNumber("Alpha Motor P Gain", 0);
  std::cout << "Read Dashboard Alpha Motor P Gain: " << p << "\n";
  i = frc::SmartDashboard::GetNumber("Alpha Motor I Gain", 0);
  std::cout << "Read Dashboard Alpha Motor I Gain: " << i << "\n";
  d = frc::SmartDashboard::GetNumber("Alpha Motor D Gain", 0);
  std::cout << "Read Dashboard Alpha Motor D Gain: " << d << "\n";
  min = frc::SmartDashboard::GetNumber("Alpha Motor Min Output", 0);
  max = frc::SmartDashboard::GetNumber("Alpha Motor Max Output", 0);

  if ((p != m_shooterAlphaCoeff.kP))
  {
    m_shooterAlphaCoeff.kP = p;
  }
  if ((i != m_shooterAlphaCoeff.kI))
  {
    m_shooterAlphaCoeff.kI = i;
  }
  if ((d != m_shooterAlphaCoeff.kD))
  {
    m_shooterAlphaCoeff.kD = d;
  }
  if ((max != m_shooterAlphaCoeff.kMaxOutput) || (min != m_shooterAlphaCoeff.kMinOutput))
  {
    m_shooterAlphaCoeff.kMinOutput = min;
    m_shooterAlphaCoeff.kMaxOutput = max;
  }

  p = frc::SmartDashboard::GetNumber("Beta Motor P Gain", 0);
  std::cout << "Read Dashboard Beta Motor P Gain: " << p << "\n";
  i = frc::SmartDashboard::GetNumber("Beta Motor I Gain", 0);
  std::cout << "Read Dashboard Beta Motor I Gain: " << i << "\n";
  d = frc::SmartDashboard::GetNumber("Beta Motor D Gain", 0);
  std::cout << "Read Dashboard Beta Motor D Gain: " << d << "\n";
  min = frc::SmartDashboard::GetNumber("Beta Motor Min Output", 0);
  max = frc::SmartDashboard::GetNumber("Beta Motor Max Output", 0);

  if ((p != m_shooterBetaCoeff.kP))
  {
    m_shooterBetaCoeff.kP = p;
  }
  if ((i != m_shooterBetaCoeff.kI))
  {
    m_shooterBetaCoeff.kI = i;
  }
  if ((d != m_shooterBetaCoeff.kD))
  {
    m_shooterBetaCoeff.kD = d;
  }
  if ((max != m_shooterBetaCoeff.kMaxOutput) || (min != m_shooterBetaCoeff.kMinOutput))
  {
    m_shooterBetaCoeff.kMinOutput = min;
    m_shooterBetaCoeff.kMaxOutput = max;
  }
}
