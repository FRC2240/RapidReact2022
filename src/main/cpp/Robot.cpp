// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"


// Standard C++ Libraries
#include <iostream>
#include <math.h>

// FIRST Specific Libraries
#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>

/**
 * Initialization method for Robot. Call Subsystem initialization methods here
 * in addition to setting up dashboard
 */
void Robot::RobotInit() {
  
  // Setup initiate Diff Drive object with an initial heading of zero
  m_odometry = new frc::DifferentialDriveOdometry(frc::Rotation2d(0_deg));
  

  InitializePIDControllers(); 
  InitializeDashboard();

  // Setup Autonomous options
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  m_chooser.AddOption(kThreeBallBlue, kThreeBallBlue);
  m_chooser.AddOption(kTwoBallBlue, kTwoBallBlue);
  m_chooser.AddOption(kThreeBallRed, kThreeBallRed);
  m_chooser.AddOption(kTwoBallRed, kTwoBallRed);

  // Add Autonomous options to dashboard
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  
  
  // Invert left side of drive train (if needed due to construction)
  m_leftDrive.SetInverted(true);
  
  // TODO: Add break ball functionality once available 

  manualShootingEnabled = true;
  wrongBallInSystem = false;

  // Setup mid motor pair
  m_midRightMotor.Follow(m_frontRightMotor);
  m_midLeftMotor.Follow(m_frontLeftMotor);

  //Setup back motor pair
  m_backRightMotor.Follow(m_frontRightMotor);
  m_backLeftMotor.Follow(m_frontLeftMotor);

  m_drive.SetSafetyEnabled(false);

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

  // Get choosen autonomous mode
  m_autoSelected = m_chooser.GetSelected();

  // Print out the selected autonomous mode
  fmt::print("Auto selected: {}\n", m_autoSelected);

  // TODO: Make the following a switch statement
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
}

/**
 * This is the method called every 20ms (by default, can be changed)
 * during the autonomous period
 */
void Robot::AutonomousPeriodic() {

  // Check if current auto mode is the custom auto mode
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }

  // Follow the defined path for autonomous
  autoFollowPath();
}

/**
 * This is the method called at the beginning of teleoperated mode
 */
void Robot::TeleopInit() {

  m_alliance = frc::DriverStation::GetAlliance();
   InitializePIDControllers();
  ReadDashboard();
}

void Robot::TeleopPeriodic() {
  // Intake
  m_take.Run(m_stick.GetLeftBumperPressed(), m_stick.GetRightBumper(), m_alliance);
  
  double a = .375/.4495;
  double b = .0745/.4495;
  //Read controller input
  //double throttle = m_stick.GetLeftTriggerAxis() - m_stick.GetRightTriggerAxis();
 
  double throttleExp = a * pow(m_stick.GetLeftTriggerAxis(), 4) + b * pow(m_stick.GetLeftTriggerAxis(), 1.48)-a * pow(m_stick.GetRightTriggerAxis(), 4) + b * pow(m_stick.GetRightTriggerAxis(), 1.48);
  double turnInput = pow(m_stick.GetLeftX()*m_turnFactor,1.72) - pow(m_stick.GetLeftY()*m_turnFactor,1.72);

  // Shooter
  if (m_stick.GetRightBumper()) {
    m_shooter.Fire();
  } else {
    m_drive.ArcadeDrive(throttleExp, turnInput);
  }
  if (m_stick.GetRightBumperReleased()) {
    m_shooter.Reset();
  }


  //climber
/*
  //move to next phase if button pressed (button subject to change)

  if (m_stick_climb.GetAButtonReleased()) { 
    m_climber.Progress();
  }

  //kill button (button subject to change)
  if (m_stick_climb.GetBButtonPressed()) {
    m_climber.Kill();
  }

  m_climber.Run(); //constantly running, initially set to zero but changes whenever progress is called
*/

  
 /*
  if (throttleExp > 1) {
    throttleExp = 1;
  } else if (throttleExp < -1) {
    throttleExp = -1;
  }*/
  //Looks like Ethan wants exponents...
   
  //TODO: climber controls



//Better Uptake
/*
  if (m_take.ManipulateBall() == m_take.rightEmpty) {} // Hold in waiting room, no shooter action needed
  if (m_take.ManipulateBall() == m_take.rightFull) {} // Hold in uptake, no shooter action needed
  // The above lines are precautionary

  // Call arcade drive function with calculated values
  m_drive.ArcadeDrive(throttleExp, turnInput);


  // TODO: climber controls

  // Semi-Autonomous Uptake Control
  // Get ManipulateBall status
  int intakeStatus = m_take.ManipulateBall();

  // TODO: Replace this with a switch statement
  // If uptake has the right ball and shooter is empty
  if (intakeStatus == m_take.rightEmpty) {
    // Hold in waiting room, no shooter action needed
  } 
  // If uptake has the right ball and shooter is full
  else if (m_take.ManipulateBall() == m_take.rightFull) {
    // Hold in uptake, no shooter action needed
  } 
  // If uptake has the wrong ball and shooter is empty
  else if (m_take.ManipulateBall() == m_take.wrongEmpty) {
    // Eject the ball
  }

  if (m_take.ManipulateBall() == m_take.wrongFull) {} // Reverse uptake, done internally 
*/
  /*
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
 }
  */

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

  // If the uptake has the wrong ball and shooter is full
  /*
  if (m_take.ManipulateBall() == m_take.wrongFull) {
    // Reverse the intake
  }  
  */

}

// This method is called at the beginning of the disabled state
void Robot::DisabledInit() {}

// This method is called every 20ms (by default) during disabled
void Robot::DisabledPeriodic() {}


// This method is called at the beginning of the testing state
void Robot::TestInit() {}

// This method is called every 20ms (by default) during testing
void Robot::TestPeriodic() {}


// Method for initializing PID Controller
void Robot::InitializePIDControllers() {
  //  m_climber.ClimberPIDInit();
  //  m_take.TakePIDInit();
  //  m_shooter.InitializePIDControllers();

}

// Method for initializing the Dashboard
void Robot::InitializeDashboard() {
  m_climber.ClimberDashInit();
  m_take.TakeDashInit();
  m_shooter.InitializeDashboard();
}

// Method for reading the Dashboard
void Robot::ReadDashboard() {
  m_climber.ClimberDashRead();
  m_take.TakeDashRead();
  m_shooter.ReadDashboard();
}

// Method for determining speeds during autonomous
void Robot::setSpeeds(const frc::DifferentialDriveWheelSpeeds& speeds) {

  // QUESTION: Why are these consts? Const in this context means you can't change
  // it after it has been set, but since these variables are local in scope and
  // not changed anywheres after that doesn't make much sense
  const auto leftFeedforward = m_feedforward.Calculate(speeds.left);
  const auto rightFeedforward = m_feedforward.Calculate(speeds.right);
 
  // QUESTION: Same as above
  const double leftOutput = m_frontRightMotorPIDController.Calculate(m_frontRightMotor.GetActiveTrajectoryVelocity(), speeds.left.to<double>());
  const double rightOutput = m_frontLeftMotorPIDController.Calculate(m_frontLeftMotor.GetActiveTrajectoryVelocity(), speeds.right.to<double>());
 
  // Set the voltages for the left and right drive motor groups
  m_leftGroup->SetVoltage(units::volt_t{leftOutput} + leftFeedforward);
  m_rightGroup->SetVoltage(units::volt_t{rightOutput} + rightFeedforward);
 
}

// QUESTION: Why is this a seperate method? Since you are just passing input
// parameters to a method and nothing else, this is just not needed
// Unless there is a plan to add more complexity/logic to THIS specfic function
void Robot::autoDrive(units::meters_per_second_t xSpeed, units::radians_per_second_t rot){
  setSpeeds(m_kinematics.ToWheelSpeeds({xSpeed, 0_mps, rot}));
}

// This method gets called every teleop period and follows the predetermined
// autonomous paths based on auto state
void Robot::autoFollowPath(){

  // QUESTION: Why all the auto types?

  // If the robot is still within the trajectory time frame
  if (autoTimer.Get() < m_trajectory.TotalTime()) {
    // Get desired pose
    auto desiredPose = m_trajectory.Sample(autoTimer.Get());
    // Get desired speeds from current pose vs desired pose
    auto refChassisSpeeds = controller1.Calculate(m_odometry->GetPose(), desiredPose);
    
    // Drive based on desired speeds
    autoDrive(refChassisSpeeds.vx, refChassisSpeeds.omega);
  }
  else {
    // Stop the robot
    autoDrive(0_mps, 0_rad_per_s);
  }
}

// If we are not running in test mode
#ifndef RUNNING_FRC_TESTS
int main() {
  // Start the robot
  return frc::StartRobot<Robot>();
}
#endif
