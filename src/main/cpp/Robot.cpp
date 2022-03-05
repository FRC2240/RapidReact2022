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
  


  m_climber.InitializeEncoders();
  m_take.InitializeEncoders(); 

  // Initialize soft limits for climber PID
  m_climber.InitializeSoftLimits();

  InitializePIDControllers(); 
  // InitializeDashboard();


//Test
m_climber.ClimberPIDInit();
  m_climber.TestDashInit();
  /*
  m_take.TestDashInit();
  m_take.TakePIDInit();
*/
  m_climber.InitializeEncoders();
  //  m_take.InitializeEncoders(); 
  m_climber.InitializeSoftLimits();
  
  // Setup Autonomous options
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  m_chooser.AddOption(kThreeBallFirst, kThreeBallFirst);
  m_chooser.AddOption(kThreeBallSecond, kThreeBallSecond);
  m_chooser.AddOption(kThreeBallThird, kThreeBallThird);
  m_chooser.AddOption(kTwoBallFirst, kTwoBallFirst);
  m_chooser.AddOption(kTwoBallSecond, kTwoBallSecond);
  
  // Add Autonomous options to dashboard
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  
  
  // Invert left side of drive train (if needed due to construction)
  m_leftDrive.SetInverted(true);
  
  // TODO: Add break ball functionality once available 

  // Enable manual shooting on initialization
  manualShootingEnabled = true;

  // Set wrong ball in system at initialization to false
  wrongBallInSystem = false;

  // Setup mid motor pair
  m_midRightMotor.Follow(m_frontRightMotor);
  m_midLeftMotor.Follow(m_frontLeftMotor);

  //Setup back motor pair
  m_backRightMotor.Follow(m_frontRightMotor);
  m_backLeftMotor.Follow(m_frontLeftMotor);

  // QUESTION: This is very unsafe. why is this here?
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
  if (m_autoSelected == kThreeBallFirst) {
    fs::path deployDirectory = frc::filesystem::GetDeployDirectory();
    deployDirectory = deployDirectory / "Paths" / "Patheaver/Paths/ThreeBallFirst.wpilib.json";
    m_trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());
  }

  if (m_autoSelected == kThreeBallSecond) {
    fs::path deployDirectory = frc::filesystem::GetDeployDirectory();
    deployDirectory = deployDirectory / "autos" / "Patheaver/autos/ThreeBallSecond.wpilib.json";
    m_trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());
  }

  if (m_autoSelected == kThreeBallThird) { 
    fs::path deployDirectory = frc::filesystem::GetDeployDirectory();
    deployDirectory = deployDirectory / "autos" / "Patheaver/autos/ThreeBallThird.wpilib.json";
    m_trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());
  }

  if (m_autoSelected == kTwoBallFirst) {
    fs::path deployDirectory = frc::filesystem::GetDeployDirectory();
    deployDirectory = deployDirectory / "autos" / "Patheaver/autos/TwoBallFirst.wpilib.json";
    m_trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());
  }

  if (m_autoSelected == kTwoBallSecond) {
    fs::path deployDirectory = frc::filesystem::GetDeployDirectory();
    deployDirectory = deployDirectory / "autos" / "Patheaver/autos/TwoBallSecond.wpilib.json";
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
  // Iteration one
  /*
   autoTimer.Start();
   if (autoTimer.Get() <= units::time::second_t(4)) {
     m_shooter.Fire();
   }
   if (autoTimer.Get() > units::time::second_t(4) && autoTimer.Get() <= units::time::second_t(5)) {
     m_drive.ArcadeDrive(0.5,0);
   }
  */

  // Iteration two
 
  autoTimer.Start();
  if (autoTimer.Get() <= units::time::second_t(1)) {
    m_take.DeployIntake();
  }
  if (autoTimer.Get() > units::time::second_t(1) && autoTimer.Get() <= units::time::second_t(5)) {
    m_take.AutoRunIntake(1);
    m_drive.ArcadeDrive(0.5, 0);
  }
  if  (autoTimer.Get() > units::time::second_t(5) && autoTimer.Get() <= units::time::second_t(9)) {
    m_shooter.Fire();;
  }
  if  (autoTimer.Get() > units::time::second_t(9) && autoTimer.Get() <= units::time::second_t(12)) {
    m_shooter.Fire();
  }
  
}

/**
 * This is the method called at the beginning of teleoperated mode
 */
void Robot::TeleopInit() {

  m_alliance = frc::DriverStation::GetAlliance();

  m_climber.ClimberPIDInit();
  m_climber.InitializeSoftLimits(); 
  //m_climber.SetPhase(0); 

}

void Robot::TeleopPeriodic() {
  m_climber.Shuffleboard();

  m_take.Run(m_stick.GetLeftBumperPressed(), m_stick.GetRightBumper(), m_alliance);
  
  /*
  double a = .375/.4495;
  double b = .0745/.4495;
  */
  //Read controller input

  double throttle = -m_stick.GetLeftTriggerAxis() + m_stick.GetRightTriggerAxis();
 
  double throttleExp = a * pow(m_stick.GetLeftTriggerAxis(), 4) + b * pow(m_stick.GetLeftTriggerAxis(), 1.48)-a * pow(m_stick.GetRightTriggerAxis(), 4) + b * pow(m_stick.GetRightTriggerAxis(), 1.48);
  // double turnInput = pow(m_stick.GetLeftX()*m_turnFactor,1.72) - pow(m_stick.GetLeftY()*m_turnFactor,1.72);
  double turnInput = m_stick.GetLeftX() - m_stick.GetLeftY();
  // Shooter
  if (m_stick.GetRightBumper()) {
    m_shooter.Fire();

  } else {
    m_drive.ArcadeDrive(throttle, turnInput);
    
    if (m_stick.GetRightBumperReleased()) {
    m_shooter.Reset();
  }

 //climber testing, JOYSTICK 1
  //fully manual 
if (m_stick_climb.GetLeftBumper()) {
      m_climber.TestL();
}
      /*
      m_climber.RotateLThrottle(0.5);
      }
    else if (m_stick_climb.GetLeftTriggerAxis()) {
      m_climber.RotateLThrottle(-0.5);
      }
      */
    else {
     m_climber.RotateLThrottle(0.0);
      //m_climber.RotateLeft(0.0);
      }

   if (m_stick_climb.GetRightBumper()) {
      m_climber.TestR(); 
   }
      /*
      m_climber.RotateRThrottle(0.5);
      }
   else if (m_stick_climb.GetRightTriggerAxis()) {
    m_climber.RotateRThrottle(-0.5);
      }
      */
   else {
    //m_climber.RotateRight(0.0);
   m_climber.RotateRThrottle(0.0);
    }

if (m_stick_climb.GetYButton()) {
  m_climber.EngageLeft(0.5);
  
}
else if (m_stick_climb.GetXButton()) {
  m_climber.EngageLeft(-0.5);
}
else {
  m_climber.EngageLeft(0.0);
}

if (m_stick_climb.GetBButton()) {
  m_climber.EngageRight(0.5);
}
else if (m_stick_climb.GetAButton()) {
  m_climber.EngageRight(-0.5);
}
else {
  m_climber.EngageRight(0.0);
}
// engage/disengage servo
if (m_stick_climb.GetLeftStickButtonReleased()) {
  if (!m_leftServoEngaged) {
    m_climber.SetLeftServo(0.0);
    m_leftServoEngaged = true;
    std::cout << "Left Servo Engaged\n";
  }
  else {
    m_climber.SetLeftServo(leftDisengaged);
    m_leftServoEngaged = false;
    std::cout << "Left Servo Disengaged\n";
  }
}

if (m_stick_climb.GetRightStickButtonReleased()) {
  if (!m_rightServoEngaged) {
    m_climber.SetRightServo(0.0);
    m_rightServoEngaged = true;
    std::cout << "Right Servo Engaged\n";
  }
  else {
    m_climber.SetRightServo(rightDisengaged);
    m_rightServoEngaged = false;
    std::cout << "Right Servo Disengaged\n";
  }
}

// This method is called at the beginning of the disabled state
void Robot::DisabledInit() {}

// This method is called every 20ms (by default) during disabled
void Robot::DisabledPeriodic() {}


// This method is called at the beginning of the testing state
void Robot::TestInit() {
}

// This method is called every 20ms (by default) during testing
void Robot::TestPeriodic() {
  m_climber.GetEncoderValues(); 
  m_climber.Run();
// JOYSTICK 0 

if (m_stick.GetXButtonReleased()) {
  m_climber.SetPhase(1);
}

if (m_stick.GetYButtonReleased()) {
  m_climber.SetPhase(2);
}

if (m_stick.GetBButtonReleased()) {
  m_climber.SetPhase(3);
}

if (m_stick.GetAButtonReleased()) {
  m_climber.SetPhase(4);
}

if (m_stick.GetLeftBumperReleased()) {
  m_climber.SetPhase(5);

}

if (m_stick.GetRightBumperReleased()) {
  m_climber.SetPhase(6);
}

if (m_stick.GetLeftTriggerAxis()) {
  m_climber.SetPhase(7);
}

if (m_stick.GetRightTriggerAxis()) {
  m_climber.SetPhase(8);
}


// This method is called at the beginning of the testing state
void Robot::TestInit() {
}

// This method is called every 20ms (by default) during testing
void Robot::TestPeriodic() {
  m_climber.GetEncoderValues(); 
  m_climber.Run();
  /*
  if (m_stick_climb.GetLeftBumperReleased()) {
    m_climber.Progress();
    std::cout << "Phase: " << m_climber.GetPhase() << "\n";
  }
  */
  
  // JOYSTICK 0 
  if (m_stick_climb.GetXButtonReleased()) {
    m_climber.SetPhase(1);
  }


  if (m_stick_climb.GetYButtonReleased()) {
    m_climber.SetPhase(2);
  }

  if (m_stick_climb.GetBButtonReleased()) {
    m_climber.SetPhase(3);
  }

  if (m_stick_climb.GetAButtonReleased()) {
    m_climber.SetPhase(4);
  }

  if (m_stick_climb.GetLeftBumperReleased()) {
    m_climber.SetPhase(5);
  }

  if (m_stick_climb.GetRightBumperReleased()) {
    m_climber.SetPhase(6);
  }

  if (m_stick_climb.GetLeftTriggerAxis()) {
    m_climber.SetPhase(7);
  }

  if (m_stick_climb.GetRightTriggerAxis()) {
    m_climber.SetPhase(8);
  }

  if (m_stick_climb.GetLeftStickButton()) {
    m_climber.SetPhase(9);
  }

  if (m_stick_climb.GetRightStickButtonReleased()) {
    m_climber.SetPhase(10); 
  }

// Method for reading the Dashboard
void Robot::ReadDashboard() {
  // m_climber.ClimberDashRead();
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
