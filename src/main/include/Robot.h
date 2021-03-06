// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>
#include <vector>

#include "Climber.h"
#include "Shooter.h"
#include "Drivetrain.h"
//#include "Take.h"

#include "rev/CANSparkMax.h"
#include "ctre/Phoenix.h"

#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

#include <wpi/sendable/Sendable.h>
#include <wpi/sendable/SendableHelper.h>

#include <frc/Filesystem.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <wpi/fs.h>


#include <frc/DriverStation.h>
#include <frc/Timer.h>
#include <frc/GenericHID.h>
#include <frc/drive/RobotDriveBase.h>
#include <frc/RobotController.h>
#include <frc/motorcontrol/MotorController.h>
#include <frc/DoubleSolenoid.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include "frc/drive/RobotDriveBase.h"
#include "frc/drive/DifferentialDrive.h"
#include <frc/XboxController.h>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include "frc/smartdashboard/SmartDashboard.h"

#include <frc/controller/PIDController.h>

#include <frc/trajectory/TrajectoryUtil.h>
#include <frc/controller/RamseteController.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/controller/SimpleMotorFeedforward.h>

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;
  void LimelightTracking();
  double CalculateRPM(double d);

  void ShooterArm();
  void ShooterFire();
  void IntakeDeploy();
  void IntakeReturn();

  void InitializePIDControllers();
  void InitializeDashboard();
  void ReadDashboard();

  bool autoFollowPath();

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoDefault = "Default";
  const std::string kMiddle      = "Middle";
  const std::string kHanger      = "Hanger";
  const std::string kTerminal    = "Terminal";
  const std::string kNoAuto      = "No Auto";
  const std::string kInstant     = "Instant";
  const std::string kDelay       = "Delay";


  std::string m_autoSelected;
  
 
  // Robot actions during Autonomous
  enum autoActions {
    kIntake,
    kShoot,
    kDump,
    kMiddlePath,
    kHangerPath,
    kTerminalPath1,
    kTerminalPath2,
    kTerminalPath3,
    kTerminalPath4,
    kInstantPath,
    kDelayPath,
    kIdle
  };

  // Robot states during Autonomous
  enum autoState {
    kDriving,
    kShooting,
    kDumping,
    kNothing,
    kInstantOK
  };

std::list<autoActions> m_middleSequence{
    kIntake,
    kMiddlePath,
    kShoot,
    kIdle
};

std::list<autoActions> m_hangerSequence{
    kIntake,
    kHangerPath,
    kShoot,
    kIdle
};

std::list<autoActions> m_terminalSequence{
    kIntake,
    kTerminalPath1,
    kTerminalPath2,
    kShoot,
    kIntake,
    kTerminalPath3,
    //kTerminalPath4,
    kShoot,
    kIdle
};

std::list<autoActions> m_instantSequence{
    kShoot,
    kInstantPath,
    kIdle
};

std::list<autoActions> m_delaySequence{
    kShoot,
    kDelayPath,
    kIdle
};

std::list<autoActions> m_noSequence{
    kIdle
};

std::list<autoActions> *m_autoSequence;
autoActions m_autoAction;
autoState m_autoState;

double m_driveExponent = 1.2;
double m_turnFactor = 0.5;
bool manualShootingEnabled;
bool limelightTrackingBool = false;
bool wrongBallInSystem;
bool uptakeBool;
fs::path deployDirectory;

double taLowBound, taHighBound;
double txLowBound, txHighBound;
double tyLowBound, tyHighBound;
double heightOfTarget;
double heightLimelight;
double constantLimelightAngle;

Climber m_climber;
Take m_take;

frc::XboxController m_stick{0};
frc::XboxController m_stick_climb{1};

WPI_TalonFX m_frontRightMotor = {8};
WPI_TalonFX m_midRightMotor = {3};
WPI_TalonFX m_backRightMotor = {7};
WPI_TalonFX m_frontLeftMotor = {2};

WPI_TalonFX m_midLeftMotor = {1};
WPI_TalonFX m_backLeftMotor = {17};

// Left side of the robot is inverted
frc::MotorControllerGroup m_leftDrive{m_frontLeftMotor, m_midLeftMotor, m_backLeftMotor};
frc::MotorControllerGroup m_rightDrive{m_frontRightMotor, m_midRightMotor, m_backRightMotor};

frc::DifferentialDrive m_drive{m_leftDrive, m_rightDrive};

frc2::PIDController m_frontRightMotorPIDController{0.0, 0.0, 0.0}; // kP, kI, kD
frc2::PIDController m_frontLeftMotorPIDController{0.0, 0.0, 0.0};

struct pidCoeff
{
  double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  };

  pidCoeff m_frontRightMotorCoeff{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  pidCoeff m_frontLeftMotorCoeff{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  
  static const int uptakeMotorDeviceID = 9;
  static const int uptakeIdleMotorDeviceID = 14;

Shooter m_shooter{&m_drive, &m_stick, &m_take};

frc::DriverStation::Alliance m_alliance = frc::DriverStation::Alliance::kInvalid;

frc::Timer m_autoTimer;

frc::Trajectory m_trajectory;

// The Ramsete Controller to follow the trajectory
frc::RamseteController m_ramseteController;

//servo toggle
bool m_leftServoEngaged = true, m_rightServoEngaged = true; 


double leftDisengaged = 0.7, rightDisengaged = 0.7;

// **** RAMSETE Control **** //
Drivetrain* m_autoDrive;

/* enabled | Limit(amp) | Trigger Threshold(amp) | Trigger Threshold Time(s) */
ctre::phoenix::motorcontrol::StatorCurrentLimitConfiguration m_statorLimit{true, 60, 60, 0.5};
ctre::phoenix::motorcontrol::SupplyCurrentLimitConfiguration m_supplyLimit{true, 60, 60, 0.5};
};
