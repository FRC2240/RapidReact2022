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
  
  InitializePIDControllers(); 
  InitializeDashboard();

  // Setup Autonomous options
  m_chooser.SetDefaultOption(kAutoDefault, kAutoDefault);
  m_chooser.AddOption(kTwoBall, kTwoBall);
  m_chooser.AddOption(kThreeBall, kThreeBall);
  
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

  m_alliance = frc::DriverStation::GetAlliance();

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
}

/**
 * This is the method called every 20ms (by default, can be changed)
 * during the autonomous period
 */
void Robot::AutonomousPeriodic() {

  fs::path deployDirectory = frc::filesystem::GetDeployDirectory();

  switch(m_autoAction) {
    case kIntake:
      m_take.Run(true, false, m_alliance);
      m_autoSequence->pop_front();
      m_autoAction = m_autoSequence->front();
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

      // Reset the drivetrain's odometry to the starting pose of the trajectory
      //m_autoDrive->ResetOdometry(m_trajectory.InitialPose());
      break;

    case kThreeBallPath1:
      deployDirectory = deployDirectory / "output/ThreeBallFirst.wpilib.json";
      m_trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());

      m_autoTimer.Reset();
      m_autoTimer.Start();
      m_autoAction = kIdle;
      m_autoState = kDriving;

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

      // Reset the drivetrain's odometry to the starting pose of the trajectory
      //m_autoDrive->ResetOdometry(m_trajectory.InitialPose());
      break;

    case kThreeBallPath3:
      deployDirectory = deployDirectory / "output/ThreeBallThird.wpilib.json";
      m_trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());

      m_autoTimer.Reset();
      m_autoTimer.Start();
      m_autoAction = kIdle;
      m_autoState = kDriving;

      // Reset the drivetrain's odometry to the starting pose of the trajectory
      //m_autoDrive->ResetOdometry(m_trajectory.InitialPose());
      break;

    case kIdle:
    default:
      m_autoState = kNothing;
      break;
  }


  if (m_autoState == kDriving) {
    bool done = autoFollowPath();

   // Next state
   if (done) {
      m_autoSequence->pop_front();
      m_autoAction = m_autoSequence->front();
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
    }
  }

  if (m_autoState == kDumping) {
    if (m_autoTimer.Get() < units::time::second_t(3)) {
      m_shooter.Fire();
    }
    else {
      m_autoSequence->pop_front();
      m_autoAction = m_autoSequence->front();
    }
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
 
  // autoTimer.Start();
  
  // if (autoTimer.Get() <= units::time::second_t(5)) {
  //   m_drive.ArcadeDrive(0.5, 0);
  // }
  // if  (autoTimer.Get() > units::time::second_t(5) && autoTimer.Get() <= units::time::second_t(9)) {
  //   m_shooter.Fire();;
  // }
  // if  (autoTimer.Get() > units::time::second_t(9) && autoTimer.Get() <= units::time::second_t(12)) {
  //   m_shooter.Fire();
  // }
  
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
  double throttle = -m_stick.GetLeftTriggerAxis() + m_stick.GetRightTriggerAxis();
 
  double throttleExp = a * pow(m_stick.GetLeftTriggerAxis(), 4) + b * pow(m_stick.GetLeftTriggerAxis(), 1.48)-a * pow(m_stick.GetRightTriggerAxis(), 4) + b * pow(m_stick.GetRightTriggerAxis(), 1.48);
  // double turnInput = pow(m_stick.GetLeftX()*m_turnFactor,1.72) - pow(m_stick.GetLeftY()*m_turnFactor,1.72);
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
