// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <iostream>

#include <fmt/core.h>

#include <math.h>

#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {
  m_climber.ClimberPIDInit();
  m_climber.ClimberDashInit();

  m_shooter.InitializePIDControllers(); 
  m_shooter.InitializeDashboard();

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
  wrongBall = false;

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

void Robot::TeleopInit() {
  m_shooter.InitializePIDControllers();
  m_shooter.ReadDashboard();

  m_climber.ClimberDashRead();
  m_climber.ClimberPIDInit();

}

void Robot::TeleopPeriodic() {

  double a = .375/.4495;
  double b = .0745/.4495;
  //Read controller input
  double throttle = m_stick.GetLeftTriggerAxis() - m_stick.GetRightTriggerAxis();
 
  double throttleExp = a * pow(throttle, 4) + b * pow(throttle, 1.48);
 
 /*
  if (throttleExp > 1) {
    throttleExp = 1;
  } else if (throttleExp < -1) {
    throttleExp = -1;
  }*/
  //Looks like Ethan wants exponents...
   
  double turnInput = m_stick.GetLeftX()*m_turnFactor - m_stick.GetLeftY()*m_turnFactor;
 
  m_drive.ArcadeDrive(throttleExp, turnInput);

  //TODO: climber controls

/*
//Better Uptake
  if (m_take.HoldBall() == 1) {} // Hold in waiting room, no shooter action needed
  if (m_take.HoldBall() == 2) {} // Hold in uptake, no shooter action needed
  // The above lines are precationary

  if (m_take.EjectBall() == 3) { // Spit out the ball
    m_shooter.Spit(0.1);
  }
  if (m_take.EjectBall() == 4) {} // Reverse uptake


 //uptake
 if (m_stick.GetAButtonPressed()) {
   if (uptakeBool == true) {
     //stop uptake
     m_take.ReturnIntake();
     uptakeBool = false;
   }
   if (uptakeBool == false) {
     //Start uptake
     m_take.DeployIntake();
     uptakeBool = true;
   }
 }*/

//Possibly uneeded
 /*
 if (m_stick.GetStartButton()){
   if (shootMan){
     shootMan = false;
     std::cout << "[MSG]: Shooter is in manual mode \n";
   }
   if (!shootMan){
     shootMan = true;
     std::cout << "[MSG]: Shooter is in automatic mode \n";
   }
 }


 if (m_stick.GetLeftBumperPressed()) {
   m_shooter.Fire();
 }
 */
}



void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::InitializePIDControllers() {
  //Climber intializes PIDs in it's own function
  m_climber.ClimberPIDInit();
  m_take.TakePIDInit();

}

void Robot::InitializeDashboard() {
  //Climbers do that in their own function
  m_climber.ClimberDashInit();
  m_take.TakeDashInit();

// Winch Motors

  /*
  if (shootMan){frc::SmartDashboard::PutNumber("Shooter Mode", "Auto");}
  if (!shootMan){frc::SmartDashboard::PutNumber("Shooter Mode", "Manual");}
  */

}

void Robot::ReadDashboard() {
  m_climber.ClimberDashRead();
  m_take.TakeDashRead();
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
