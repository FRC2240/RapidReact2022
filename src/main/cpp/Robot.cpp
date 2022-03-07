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

  // InitializePIDControllers(); 
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
  m_chooser.SetDefaultOption(kAutoDefault, kAutoDefault);
  //m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  m_chooser.AddOption(Robot::kTwoBall, Robot::kTwoBall);
  m_chooser.AddOption(Robot::kThreeBall, Robot::kThreeBall);
  
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

  // Initialize auto driver
  m_autoDrive = new Drivetrain(&m_leftDrive, &m_rightDrive, &m_frontLeftMotor, &m_frontRightMotor);
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

  if (m_autoSelected == kTwoBall) {
    m_autoSequence = &m_twoBallSequence;
  }

  if (m_autoSelected == kThreeBall) {
    m_autoSequence = &m_threeBallSequence;
  }

  if (m_autoSelected == kAutoDefault) {
    m_autoSequence = &m_noSequence;
  }

  // First action
  m_autoAction = m_autoSequence->front();
  m_autoState = kNothing;
}

/**
 * This is the method called every 20ms (by default, can be changed)
 * during the autonomous period
 */
void Robot::AutonomousPeriodic() {

  fs::path deployDirectory = frc::filesystem::GetDeployDirectory();

  // Execute action
  switch(m_autoAction) {
    case kIntake:
      m_take.Run(true, false, true, m_alliance);
      m_autoSequence->pop_front();
      m_autoAction = m_autoSequence->front();
      m_autoState = kNothing;
      break;

    case kShoot:
      m_autoTimer.Reset();
      m_autoTimer.Start();
      m_autoAction = kIdle;
      m_autoState = kShooting;
      break;

    case kDump:
      m_autoTimer.Reset();
      m_autoTimer.Start();
      m_autoAction = kIdle;
      m_autoState = kDumping;
      break;

    case kTwoBallPath1:
      deployDirectory = deployDirectory / "output/TwoBallFirst.wpilib.json";
      m_trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());

      m_autoTimer.Reset();
      m_autoTimer.Start();
      m_autoAction = kIdle;
      m_autoState = kDriving;

      m_autoDrive->ResetEncoders();

      // Reset the drivetrain's odometry to the starting pose of the trajectory
      m_autoDrive->ResetOdometry(m_trajectory.InitialPose());
      break;

    case kTwoBallPath2:
      deployDirectory = deployDirectory / "output/TwoBallSecond.wpilib.json";
      m_trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());

      m_autoTimer.Reset();
      m_autoTimer.Start();
      m_autoAction = kIdle;
      m_autoState = kDriving;

      m_autoDrive->ResetEncoders();

      // Reset the drivetrain's odometry to the starting pose of the trajectory
      m_autoDrive->ResetOdometry(m_trajectory.InitialPose());
      break;

    case kThreeBallPath1:
      deployDirectory = deployDirectory / "output/ThreeBallFirst.wpilib.json";
      m_trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());

      m_autoTimer.Reset();
      m_autoTimer.Start();
      m_autoAction = kIdle;
      m_autoState = kDriving;

      m_autoDrive->ResetEncoders();

      // Reset the drivetrain's odometry to the starting pose of the trajectory
      m_autoDrive->ResetOdometry(m_trajectory.InitialPose());
      break;
      
    case kThreeBallPath2:
      deployDirectory = deployDirectory / "output/ThreeBallSecond.wpilib.json";
      m_trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());

      m_autoTimer.Reset();
      m_autoTimer.Start();
      m_autoAction = kIdle;
      m_autoState = kDriving;

      m_autoDrive->ResetEncoders();

      // Reset the drivetrain's odometry to the starting pose of the trajectory
      m_autoDrive->ResetOdometry(m_trajectory.InitialPose());
      break;

    case kThreeBallPath3:
      deployDirectory = deployDirectory / "output/ThreeBallThird.wpilib.json";
      m_trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());

      m_autoTimer.Reset();
      m_autoTimer.Start();
      m_autoAction = kIdle;
      m_autoState = kDriving;

      m_autoDrive->ResetEncoders();

      // Reset the drivetrain's odometry to the starting pose of the trajectory
      m_autoDrive->ResetOdometry(m_trajectory.InitialPose());
      break;

    case kIdle:
    default:
      break;
  }

  // Long-lived states...
  if (m_autoState == kDriving) {
    bool done = autoFollowPath();
    m_take.Run(false, false, true, m_alliance);

   // Next state
   if (done) {
      m_take.Run(true, true, true, m_alliance);
      m_autoSequence->pop_front();
      m_autoAction = m_autoSequence->front();
      m_autoState = kNothing;
    }
  }

  if (m_autoState == kShooting) {
    if (m_autoTimer.Get() < units::time::second_t(4)) {
      m_shooter.Fire();
    }
    else {
      m_shooter.Reset();
      m_autoSequence->pop_front();
      m_autoAction = m_autoSequence->front();
      m_autoState = kNothing;
    }
  }

  if (m_autoState == kDumping) {
    if (m_autoTimer.Get() < units::time::second_t(3)) {
      m_shooter.Dump();
    }
    else {
      m_shooter.Reset();
      m_autoSequence->pop_front();
      m_autoAction = m_autoSequence->front();
      m_autoState = kNothing;
    }
  }
  // Iteration two
 
  /*Robot::m_autoTimer.Start();
  if (Robot::m_autoTimer.Get() <= units::time::second_t(1)) {
    m_take.DeployIntake();
  }
  if (Robot::m_autoTimer.Get() > units::time::second_t(1) && Robot::m_autoTimer.Get() <= units::time::second_t(5)) {
    m_take.AutoRunIntake(1);
    m_drive.ArcadeDrive(0.5, 0);
  }
  if  (Robot::m_autoTimer.Get() > units::time::second_t(5) && Robot::m_autoTimer.Get() <= units::time::second_t(9)) {
    m_shooter.Fire();
  }
  if  (Robot::m_autoTimer.Get() > units::time::second_t(9) && Robot::m_autoTimer.Get() <= units::time::second_t(12)) {
    m_shooter.Fire();
  }*/
}

/**
 * This is the method called at the beginning of teleoperated mode
 */
void Robot::TeleopInit() {

  m_alliance = frc::DriverStation::GetAlliance();
  //InitializePIDControllers();
  ReadDashboard();
}

void Robot::TeleopPeriodic() {
  // Intake
  m_climber.Run();
  m_take.Run(m_stick.GetLeftBumperReleased(), m_stick.GetRightBumper(), false, m_alliance);
  
  double a = .375/.4495;
  double b = .0745/.4495;
  //Read controller input

  double throttle = -m_stick.GetLeftTriggerAxis() + m_stick.GetRightTriggerAxis();

  double turnInput = m_stick.GetLeftX() - m_stick.GetLeftY();
  // Shooter
  if (m_stick.GetRightBumper()) {
    m_shooter.Fire();
  } else {
    m_drive.ArcadeDrive(throttle, turnInput);
  }
  
  if (m_stick.GetRightBumperReleased()) {
    m_shooter.Reset();
  }
  
  // Climb Controls (Joystick 1)
  if (m_stick_climb.GetLeftBumperReleased()) {
    m_climber.Progress();
  }

  if (m_stick_climb.GetYButtonReleased()) {
    m_climber.SetPhase(6); 
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

  // Manual Climb Controls
  // Arm extension
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

  //Arm rotation
  if (m_stick_climb.GetLeftBumper()) {
    m_climber.RotateLThrottle(0.5);
  }
  else if (m_stick_climb.GetLeftTriggerAxis()) {
    m_climber.RotateLThrottle(-0.5);
  }
  else {
    m_climber.RotateLThrottle(0.0);
  }

  if (m_stick_climb.GetRightBumper()) {
    m_climber.RotateRThrottle(0.5);
  }
  else if (m_stick_climb.GetRightTriggerAxis()) {
    m_climber.RotateRThrottle(-0.5);
  }
  else {
    m_climber.RotateRThrottle(0.0);
  }

  // engage/disengage servos
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
}

// Method for reading the Dashboard
void Robot::ReadDashboard() {
  m_climber.ClimberDashRead();
  m_take.TakeDashRead();
  m_shooter.ReadDashboard();
}

bool Robot::autoFollowPath()
{
  // Update odometry
  m_autoDrive->UpdateOdometry();

  if (m_autoTimer.Get() < m_trajectory.TotalTime()) {
    // Get the desired pose from the trajectory
    auto desiredPose = m_trajectory.Sample(m_autoTimer.Get());

    // Get the reference chassis speeds from the Ramsete Controller
    // std::cout << "x = " << m_drive->GetPose().X()
    //          <<  "y = " << m_drive->GetPose().Y() << " rot = " << m_drive->GetPose().Rotation().Degrees() << std::endl;
    // std::cout << "dx = " << desiredPose.pose.X()
    //          << " dy = " << desiredPose.pose.Y() << " drot = " << desiredPose.pose.Rotation().Degrees() << std::endl;

    auto refChassisSpeeds = m_ramseteController.Calculate(m_autoDrive->GetPose(), desiredPose);

    // Set the linear and angular speeds
    m_autoDrive->Drive(refChassisSpeeds.vx, refChassisSpeeds.omega);

    return false;
  }
  else {
    m_autoDrive->Drive(0_mps, 0_rad_per_s);
    return true;
  }
}


// If we are not running in test mode
#ifndef RUNNING_FRC_TESTS
int main() {
  // Start the robot
  return frc::StartRobot<Robot>();
}
#endif
