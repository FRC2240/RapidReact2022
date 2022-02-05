// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <iostream>

#include <fmt/core.h>

#include <math.h>

#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  m_chooser.AddOption(kThreeBallBlue, kThreeBallBlue);
  m_chooser.AddOption(kTwoBallBlue, kTwoBallBlue);
  m_chooser.AddOption(kThreeBallRed, kThreeBallRed);
  m_chooser.AddOption(kTwoBallRed, kTwoBallRed);

  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
// right side might need to be inverted depending on construction
  m_leftDrive.SetInverted(true);
  //  double ballsInShooter = 0; //add when break bar functionality is added
  shootMan = true;

}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {

  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  fmt::print("Auto selected: {}\n", m_autoSelected);

  if (m_autoSelected == kThreeBallBlue) {
    fs::path deployDirectory = frc::filesystem::GetDeployDirectory();
    deployDirectory = deployDirectory / "autos" / "Patheaver/autos/ThreeBallBlue.wpilib.json";
    m_trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());
  }

  if (m_autoSelected == kTwoBallBlue) {
    fs::path deployDirectory = frc::filesystem::GetDeployDirectory();
    deployDirectory = deployDirectory / "autos" / "Patheaver/autos/TwoBallBlue.wpilib.json";
    m_trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());
  }

  if (m_autoSelected == kThreeBallRed) { 
    fs::path deployDirectory = frc::filesystem::GetDeployDirectory();
    deployDirectory = deployDirectory / "autos" / "Patheaver/autos/ThreeBallRed.wpilib.json";
    m_trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());
  }

  if (m_autoSelected == kTwoBallRed) {
    fs::path deployDirectory = frc::filesystem::GetDeployDirectory();
    deployDirectory = deployDirectory / "autos" / "Patheaver/autos/TwoBallRed.wpilib.json";
    m_trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());
  }
      // Custom Auto goes here
  else {
      // Default Auto goes here
    }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }

  // autoFollowPath();
  // Iteration one
  /*
   autoTimer.Start();
   if (autoTimer.Get() <= units::time::second_t(4)) {
     LimelightTracking();
     ShooterArm();
     ShooterFire();
   }
   if (autoTimer.Get() > units::time::second_t(4) && autoTimer.Get() <= units::time::second_t(5)) {
     m_drive.ArcadeDrive(0.5,0);
   }
*/

  // Iteration two
  /*
  autoTimer.Start();
  if (autoTimer.Get() <= units::time::second_t(0.5)) {
  }
  if (autoTimer.Get() > units::time::second_t(0.5) && autoTimer.Get() <= units::time::second_t(5)) {
    m_drive.ArcadeDrive(0.5, 0);
  }
  if  (autoTimer.Get() > units::time::second_t(5) && autoTimer.Get() <= units::time::second_t(8)) {
    m_drive.ArcadeDrive(0, 0.5);
  }
  if  (autoTimer.Get() > units::time::second_t(8) && autoTimer.Get() <= units::time::second_t(12)) {
    LimelightTracking();
  }
  if  (autoTimer.Get() > units::time::second_t(12) && autoTimer.Get() <= units::time::second_t(15)) {
    LimelightTracking();
  }
  */

  // Iteration three
  /*
  autoTimer.Start();
  if (autoTimer.Get() <= units::time::second_t(4)) {
    LimelightTracking();
    ShooterArm();
    ShooterFire();
  }
  if (autoTimer.Get() > units::time::second_t(4) && autoTimer.Get() <= units::time::second_t(8)) {

  }
  ready aim and fire the two extra balls
  */
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {

  //Read controller input

  double throttle = m_stick.GetRightTriggerAxis() - m_stick.GetLeftTriggerAxis();

  //Looks like Ethan wants exponents...
  double throttleExp = pow(throttle, m_driveExponent);
  double turnInput = pow(m_stick.GetRightX(), m_driveExponent);

  m_drive.ArcadeDrive(throttleExp, turnInput);


  //Intake
  

  //uptake
}



// Ready!
void Robot::LimelightTracking() {
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
        limelightTrackingBool = true;
    }
    else {
        limelightTrackingBool = false;
    }

  //If it's tracking, use limebool
}


void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::InitializePIDControllers() {
  //Climber intializes PIDs in it's own function
  m_climber.ClimberPIDInit();
  m_take.TakePIDInit();
  
  //winch motors
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

void Robot::InitializeDashboard() {
  //Climbers do that in their own function
  m_climber.ClimberDashInit();
  m_take.TakeDashInit();

// Winch Motors
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

  /*
  if (shootMan){frc::SmartDashboard::PutNumber("Shooter Mode", "Auto");}
  if (!shootMan){frc::SmartDashboard::PutNumber("Shooter Mode", "Manual");}
  */

}

void Robot::ReadDashboard() {
  double p, i, d, min, max;
  m_climber.ClimberDashRead();
  m_take.TakeDashRead();

  // Shooting motors

  p   = frc::SmartDashboard::GetNumber("Alpha Motor P Gain", 0);
  std::cout << "Read Dashboard Alpha Motor P Gain: " << p << "\n";
  i   = frc::SmartDashboard::GetNumber("Alpha Motor I Gain", 0);
  std::cout << "Read Dashboard Alpha Motor I Gain: " << i << "\n";
  d   = frc::SmartDashboard::GetNumber("Alpha Motor D Gain", 0);
  std::cout << "Read Dashboard Alpha Motor D Gain: " << d << "\n";
  min = frc::SmartDashboard::GetNumber("Alpha Motor Min Output", 0);
  max = frc::SmartDashboard::GetNumber("Alpha Motor Max Output", 0);

  if ((p != m_shooterAlphaCoeff.kP)) { m_shooterAlphaPIDController.SetP(p);m_shooterAlphaCoeff.kP = p; }
  if ((i != m_shooterAlphaCoeff.kI)) { m_shooterAlphaPIDController.SetI(i); m_shooterAlphaCoeff.kI = i; }
  if ((d != m_shooterAlphaCoeff.kD)) { m_shooterAlphaPIDController.SetD(d); m_shooterAlphaCoeff.kD = d; }
  if ((max != m_shooterAlphaCoeff.kMaxOutput) || (min != m_shooterAlphaCoeff.kMinOutput)) { 
    m_shooterAlphaPIDController.SetOutputRange(min, max); 
    m_shooterAlphaCoeff.kMinOutput = min; m_shooterAlphaCoeff.kMaxOutput = max; 
  }

   p   = frc::SmartDashboard::GetNumber("Beta Motor P Gain", 0);
  std::cout << "Read Dashboard Beta Motor P Gain: " << p << "\n";
  i   = frc::SmartDashboard::GetNumber("Beta Motor I Gain", 0);
  std::cout << "Read Dashboard Beta Motor I Gain: " << i << "\n";
  d   = frc::SmartDashboard::GetNumber("Beta Motor D Gain", 0);
  std::cout << "Read Dashboard Beta Motor D Gain: " << d << "\n";
  min = frc::SmartDashboard::GetNumber("Beta Motor Min Output", 0);
  max = frc::SmartDashboard::GetNumber("Beta Motor Max Output", 0);

  if ((p != m_shooterBetaCoeff.kP)) { m_shooterBetaPIDController.SetP(p);m_shooterBetaCoeff.kP = p; }
  if ((i != m_shooterBetaCoeff.kI)) { m_shooterBetaPIDController.SetI(i); m_shooterBetaCoeff.kI = i; }
  if ((d != m_shooterBetaCoeff.kD)) { m_shooterBetaPIDController.SetD(d); m_shooterBetaCoeff.kD = d; }
  if ((max != m_shooterBetaCoeff.kMaxOutput) || (min != m_shooterBetaCoeff.kMinOutput)) { 
    m_shooterBetaPIDController.SetOutputRange(min, max); 
    m_shooterBetaCoeff.kMinOutput = min; m_shooterBetaCoeff.kMaxOutput = max; 
  }
}

// void Robot::setSpeeds(const frc::DifferentialDriveWheelSpeeds& speeds) {
//   const auto leftFeedforward = m_feedforward.Calculate(speeds.left);
//   const auto rightFeedforward = m_feedforward.Calculate(speeds.right);


// }

// void Robot::autoDrive(units::meters_per_second_t xSpeed, units::radians_per_second_t rot){
//   setSpeeds(m_kinematics.ToWheelSpeeds({xSpeed, 0_mps, rot}));
// }

// void Robot::autoFollowPath(){
//   if (autoTimer.Get() < m_trajectory.TotalTime()) {
//     auto desiredPose = m_trajectory.Sample(autoTimer.Get());
//     auto refChassisSpeeds = controller1.Calculate(m_odometry.GetPose(), desiredPose);
    
//   }
// }

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
