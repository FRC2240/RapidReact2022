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

  m_climber.ClimberPIDInit();

  m_climber.InitializeEncoders();
  m_climber.InitializeSoftLimits();
  
  // Setup Autonomous options
  m_chooser.AddOption(Robot::kMiddle, Robot::kMiddle);
  m_chooser.AddOption(Robot::kHanger, Robot::kHanger);
  m_chooser.AddOption(Robot::kTerminal, Robot::kTerminal);
  m_chooser.AddOption(Robot::kNoAuto, Robot::kNoAuto);
  m_chooser.AddOption(Robot::kInstant, Robot::kInstant);
  m_chooser.AddOption(Robot::kDelay, Robot::kDelay);
  
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

  // Apply current limits
  m_frontRightMotor.ConfigStatorCurrentLimit(m_statorLimit);
    m_midRightMotor.ConfigStatorCurrentLimit(m_statorLimit);
   m_backRightMotor.ConfigStatorCurrentLimit(m_statorLimit);

   m_frontLeftMotor.ConfigStatorCurrentLimit(m_statorLimit);
     m_midLeftMotor.ConfigStatorCurrentLimit(m_statorLimit);
    m_backLeftMotor.ConfigStatorCurrentLimit(m_statorLimit);

  m_frontRightMotor.ConfigSupplyCurrentLimit(m_supplyLimit);
    m_midRightMotor.ConfigSupplyCurrentLimit(m_supplyLimit);
   m_backRightMotor.ConfigSupplyCurrentLimit(m_supplyLimit);

   m_frontLeftMotor.ConfigSupplyCurrentLimit(m_supplyLimit);
     m_midLeftMotor.ConfigSupplyCurrentLimit(m_supplyLimit);
    m_backLeftMotor.ConfigSupplyCurrentLimit(m_supplyLimit);
      
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
  //fmt::print("Auto selected: {}\n", m_autoSelected);

  if (m_autoSelected == kMiddle) {
    m_autoSequence = &m_middleSequence;
  }

  if (m_autoSelected == kHanger) {
    m_autoSequence = &m_hangerSequence;
  }

  if (m_autoSelected == kTerminal) {
    m_autoSequence = &m_terminalSequence;
  }

  if (m_autoSelected == kNoAuto) {
    m_autoSequence = &m_noSequence;
  }

  if (m_autoSelected == kInstant) {
    m_autoSequence = &m_instantSequence;
  }

  if (m_autoSelected == kDelay) {
    m_autoSequence = &m_delaySequence;
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

    case kMiddlePath:
      deployDirectory = deployDirectory / "output/MiddlePath.wpilib.json";
      m_trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());

      m_autoTimer.Reset();
      m_autoTimer.Start();
      m_autoAction = kIdle;
      m_autoState = kDriving;

      m_autoDrive->ResetEncoders();

      // Reset the drivetrain's odometry to the starting pose of the trajectory
      m_autoDrive->ResetOdometry(m_trajectory.InitialPose());
      break; 

    case kHangerPath:
      deployDirectory = deployDirectory / "output/HangerPath.wpilib.json";
      m_trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());

      m_autoTimer.Reset();
      m_autoTimer.Start();
      m_autoAction = kIdle;
      m_autoState = kDriving;

      m_autoDrive->ResetEncoders();

      // Reset the drivetrain's odometry to the starting pose of the trajectory
      m_autoDrive->ResetOdometry(m_trajectory.InitialPose());
      break;

    case kTerminalPath1:
      deployDirectory = deployDirectory / "output/TerminalPath1.wpilib.json";
      m_trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());

      m_autoTimer.Reset();
      m_autoTimer.Start();
      m_autoAction = kIdle;
      m_autoState = kDriving;

      m_autoDrive->ResetEncoders();

      // Reset the drivetrain's odometry to the starting pose of the trajectory
      m_autoDrive->ResetOdometry(m_trajectory.InitialPose());
      break;

    case kTerminalPath2:
      deployDirectory = deployDirectory / "output/TerminalPath2.wpilib.json";
      m_trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());

      m_autoTimer.Reset();
      m_autoTimer.Start();
      m_autoAction = kIdle;
      m_autoState = kDriving;

      m_autoDrive->ResetEncoders();

      // Reset the drivetrain's odometry to the starting pose of the trajectory
      m_autoDrive->ResetOdometry(m_trajectory.InitialPose());
      break;

    case kTerminalPath3:
      deployDirectory = deployDirectory / "output/TerminalPath3.wpilib.json";
      m_trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());

      m_autoTimer.Reset();
      m_autoTimer.Start();
      m_autoAction = kIdle;
      m_autoState = kDriving;

      m_autoDrive->ResetEncoders();

      // Reset the drivetrain's odometry to the starting pose of the trajectory
      m_autoDrive->ResetOdometry(m_trajectory.InitialPose());
      break;

    case kTerminalPath4:
      deployDirectory = deployDirectory / "output/TerminalPath4.wpilib.json";
      m_trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());

      m_autoTimer.Reset();
      m_autoTimer.Start();
      m_autoAction = kIdle;
      m_autoState = kDriving;

      m_autoDrive->ResetEncoders();

      // Reset the drivetrain's odometry to the starting pose of the trajectory
      m_autoDrive->ResetOdometry(m_trajectory.InitialPose());
      break;

    case kInstantPath:
      m_autoTimer.Reset();
      m_autoTimer.Start();
      m_drive.ArcadeDrive(-0.5, 0);
      m_autoAction = kIdle; 
      m_autoState = kInstantOK;       

      break;

      case kDelayPath:

      m_autoTimer.Reset();
      m_autoTimer.Start();
      m_autoAction = kIdle;
      m_autoState = kDriving;

      if (m_autoTimer.Get() > units::time::second_t(7) && m_autoTimer.Get() < units::time::second_t(8.5)) {
        m_drive.ArcadeDrive(-0.3, 0);
      } else {
        m_drive.ArcadeDrive(0, 0);
      }

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

 if (m_autoState == kInstantOK) {
   if (m_autoTimer.Get() > units::time::second_t(1.5)) {
     m_drive.ArcadeDrive(0,0); 
   }
 }

  if (m_autoState == kShooting) {
    if (m_autoTimer.Get() < units::time::second_t(8)) {
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
    if (m_autoTimer.Get() < units::time::second_t(2.0)) {
      m_shooter.Dump();
    }
    else {
      m_shooter.Reset();
      m_autoSequence->pop_front();
      m_autoAction = m_autoSequence->front();
      m_autoState = kNothing;
    }
  }
}

/**
 * This is the method called at the beginning of teleoperated mode
 */
void Robot::TeleopInit() {
  m_alliance = frc::DriverStation::GetAlliance();
  //m_shooter.ReadDashboard();
  m_shooter.InitializePIDControllers();
  //ReadDashboard();
}

void Robot::TeleopPeriodic() {
  // Intake
  m_take.Run(m_stick.GetLeftBumperReleased(), m_stick.GetRightBumper(), false, m_alliance);
  
  //double a = .375/.4495;
  //double b = .0745/.4495;

  // Read controller input
  double throttle = -m_stick.GetLeftTriggerAxis() + m_stick.GetRightTriggerAxis();

  double turnInput = m_stick.GetLeftX();
  // Shooter
  // each time x is pressed, RPM increased by 10
  if (m_stick_climb.GetXButtonReleased()) {
    m_shooter.m_scalar++;
  }
  // each time a is pressed, RPM is decreased by 10
  if (m_stick_climb.GetAButtonReleased()) {
    m_shooter.m_scalar--; 
  }
  if (m_stick.GetAButton()) {
    m_shooter.Go(); //manual override
  }
  if (m_stick.GetRightBumper()) {
    m_shooter.Fire();
  } else {
    m_drive.ArcadeDrive(throttle, turnInput);
  }
  if (m_stick.GetRightBumperReleased() || m_stick.GetAButtonReleased()) {
    m_shooter.Reset();
  }

  double climbThrottle = m_stick_climb.GetLeftY() * 0.5;
  m_climber.EngageLeft(-climbThrottle);
//WARING: NEVER USE 0.4
//IT WILL STALL AND NOT WORK. I don't know why.

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
