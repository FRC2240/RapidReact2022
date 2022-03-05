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
  
  m_climber.InitializeEncoders();
  m_take.InitializeEncoders(); 

  // Initialize soft limits for climber PID
  m_climber.InitializeSoftLimits();

  //Robot::InitializePIDControllers(); 
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
  m_chooser.AddOption(kTwoBall, kTwoBall);
  m_chooser.AddOption(kThreeBall, kThreeBall);
  
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
      std::cout << "Intake\n";
      m_take.Run(true, false, m_alliance);
      m_autoSequence->pop_front();
      m_autoAction = m_autoSequence->front();
      std::cout << "new = " << m_autoAction << "\n";
      m_autoState = kNothing;
      break;

    case kShoot:
      std::cout << "Shoot\n";

      m_autoTimer.Reset();
      m_autoTimer.Start();
      m_autoAction = kIdle;
      m_autoState = kShooting;
      break;

    case kDump:
      std::cout << "Dump\n";

      m_autoTimer.Reset();
      m_autoTimer.Start();
      m_autoAction = kIdle;
      m_autoState = kDumping;
      break;

    case kTwoBallPath1:
      std::cout << "Two Ball Path 1\n";

      deployDirectory = deployDirectory / "output/TwoBallFirst.wpilib.json";
      m_trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());
      std::cout << "dir: " << deployDirectory.string() << std::endl;

      std::cout << "Trajectory time: " << m_trajectory.TotalTime().to<double>() << std::endl;

      m_autoTimer.Reset();
      m_autoTimer.Start();
      m_autoAction = kIdle;
      m_autoState = kDriving;

      // Reset the drivetrain's odometry to the starting pose of the trajectory
      m_autoDrive->ResetOdometry(m_trajectory.InitialPose());
      break;

    case kTwoBallPath2:
      std::cout << "Two Ball Path 2\n";

      deployDirectory = deployDirectory / "output/TwoBallSecond.wpilib.json";
      m_trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());

      std::cout << "Trajectory time: " << m_trajectory.TotalTime().to<double>() << std::endl;

      m_autoTimer.Reset();
      m_autoTimer.Start();
      m_autoAction = kIdle;
      m_autoState = kDriving;

      // Reset the drivetrain's odometry to the starting pose of the trajectory
      m_autoDrive->ResetOdometry(m_trajectory.InitialPose());
      break;

    case kThreeBallPath1:
      std::cout << "Three Ball Path 1\n";

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
      std::cout << "Three Ball Path 2\n";

      deployDirectory = deployDirectory / "output/ThreeBallSecond.wpilib.json";
      m_trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());

      m_autoTimer.Reset();
      m_autoTimer.Start();
      m_autoAction = kIdle;
      m_autoState = kDriving;

      // Reset the drivetrain's odometry to the starting pose of the trajectory
      m_autoDrive->ResetOdometry(m_trajectory.InitialPose());
      break;

    case kThreeBallPath3:
      std::cout << "Three Ball Path 3\n";

      deployDirectory = deployDirectory / "output/ThreeBallThird.wpilib.json";
      m_trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());

      m_autoTimer.Reset();
      m_autoTimer.Start();
      m_autoAction = kIdle;
      m_autoState = kDriving;

      // Reset the drivetrain's odometry to the starting pose of the trajectory
      m_autoDrive->ResetOdometry(m_trajectory.InitialPose());
      break;

    case kIdle:
    default:
      //std::cout << "Default/Idle\n";
      break;
  }

  // Long-lived states...
  if (m_autoState == kDriving) {
    //std::cout << "driving" << std::endl;
    bool done = autoFollowPath();

   // Next state
   if (done) {
      std::cout << "done driving" << std::endl;
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
      std::cout << "shoot done\n";
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
      std::cout << "dump done\n";
      m_shooter.Reset();
      m_autoSequence->pop_front();
      m_autoAction = m_autoSequence->front();
      m_autoState = kNothing;
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
 
 /*
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
  */
  
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
  
}

// This method is called at the beginning of the disabled state
void Robot::DisabledInit() {}

// This method is called every 20ms (by default) during disabled
void Robot::DisabledPeriodic() {}


// This method is called at the beginning of the testing state


// This method is called every 20ms (by default) during testing
void Robot::TestPeriodic() {
  std::cout << " left: " << m_frontLeftMotor.GetSelectedSensorPosition() << std::endl;
  std::cout << "right: " << m_frontRightMotor.GetSelectedSensorPosition() << std::endl;

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
}


// This method is called at the beginning of the testing state
void Robot::TestInit() {
  m_frontLeftMotor.SetSelectedSensorPosition(0.0);
  m_frontRightMotor.SetSelectedSensorPosition(0.0);
}

/* Commented due to duplication. Uncomment if needed */

/*
// This method is called every 20ms (by default) during testing
void Robot::TestPeriodic() {
  m_climber.GetEncoderValues(); 
  m_climber.Run();
  
  if (m_stick_climb.GetLeftBumperReleased()) {
    m_climber.Progress();
    std::cout << "Phase: " << m_climber.GetPhase() << "\n";
  }
  
  
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
}
*/
// Method for reading the Dashboard
void Robot::ReadDashboard() {
  // m_climber.ClimberDashRead();
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
    //std::cout << "did drive" << std::endl;

    return false;
  }
  else {
    m_autoDrive->Drive(0_mps, 0_rad_per_s);
        //std::cout << "done drive" << std::endl;

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
