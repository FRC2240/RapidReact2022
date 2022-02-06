#include "Shooter.h"

#include <iostream>

#include <fmt/core.h>

#include <math.h>

#include <frc/smartdashboard/SmartDashboard.h>

bool Shooter::limelightTracking(){
  nt::NetworkTableEntry txEntry;
  nt::NetworkTableEntry tyEntry;
  nt::NetworkTableEntry taEntry;

  double ta, tx, ty;

  auto inst = nt::NetworkTableInstance::GetDefault();
  auto table = inst.GetTable("limelight-bepis");
  txEntry = table->GetEntry("tx");
  tyEntry = table->GetEntry("ty");
  taEntry = table->GetEntry("ta");


  txEntry.SetDouble(tx);
  tyEntry.SetDouble(ty);
  taEntry.SetDouble(ta);

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
  else {
    return false;
  }
}

double Shooter::CalculateRPM(double d){}

void Shooter::ShooterArm(){}

void Shooter::ShooterFire(){
 if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed){
   if (m_take.BallColorRoom() == 'r') {wrongBall = false;}
   if (m_take.BallColorRoom() == 'b') {wrongBall = true;}
   if (m_take.BallColorRoom() == 'E') {std::cout << "[WARN]: Color Sensor issue \n";}
  }

  if (frc::DriverStation::GetAlliance()  == frc::DriverStation::Alliance::kBlue){
    if (m_take.BallColorRoom() == 'r') {wrongBall = true;}
    if (m_take.BallColorRoom() == 'b') {wrongBall = false;}
    if (m_take.BallColorRoom() == 'E') {std::cout << "[WARN]: Color Sensor issue \n";}
}

if (wrongBall){
  //  sosTimer.Start()
  // Make an SOS
  m_stick.SetRumble(frc::GenericHID::RumbleType::kLeftRumble, 1.0);
  m_stick.SetRumble(frc::GenericHID::RumbleType::kRightRumble, 1.0);
 }
 if (!wrongBall){
   m_stick.SetRumble(frc::GenericHID::RumbleType::kLeftRumble, 0.0);
   m_stick.SetRumble(frc::GenericHID::RumbleType::kRightRumble, 0.0);
 }

  nt::NetworkTableEntry txEntry;
  nt::NetworkTableEntry tyEntry;
  nt::NetworkTableEntry taEntry;

  double ta, tx, ty;

  auto inst = nt::NetworkTableInstance::GetDefault();
  auto table = inst.GetTable("limelight-bepis");
  txEntry = table->GetEntry("tx");
  tyEntry = table->GetEntry("ty");
  taEntry = table->GetEntry("ta");


  txEntry.SetDouble(tx);
  tyEntry.SetDouble(ty);
  taEntry.SetDouble(ta);

  ty = ty * -1;

  if (shootMan){
    if (Shooter::LimelightTracking()  == true) {
      //Code stolen. Procedure is to map ty to theta, subtract a value I forget and then you get your angle. Solve from there.

      double distance = ((heightOfTarget - heightLimelight) / tan((constantLimelightAngle + ty) * (3.141592653 / 180)));

      double rpm = CalculateRPM(distance);
      m_shooterAlphaPIDController.SetReference(rpm, rev::ControlType::kVelocity);
      m_shooterBetaPIDController.SetReference(rpm, rev::ControlType::kVelocity);
}
    else {
            if (tx < 0){
                        m_drive.ArcadeDrive(0, 0.5);
            }
            if (tx > 0){
                        m_drive.ArcadeDrive(0, -0.5);
            }
      }
  }

  if (!shootMan){
    //Code stolen. Procedure is to map ty to theta, subtract a value I forget and then you get your angle. Solve from there.
                 double distance = ((heightOfTarget - heightLimelight) / tan((constantLimelightAngle + ty) * (3.141592653 / 180)));

                 double rpm = CalculateRPM(distance);
                 m_shooterAlphaPIDController.SetReference(rpm, rev::ControlType::kVelocity);
                 m_shooterBetaPIDController.SetReference(rpm, rev::ControlType::kVelocity);
  }}

void Shooter::InitializePIDControllers(){}

void Shooter::InitializeDashboard(){}

void Shooter::ReadDashboard(){}

