// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include "rev/CANSparkMax.h"
#include "ctre/Phoenix.h"

#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

#include <wpi/sendable/Sendable.h>
#include <wpi/sendable/SendableHelper.h>

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
  void ShooterFire();
  void LimelightTracking();
  void ShooterArm();
  void IntakeDeploy();
  void IntakeReturn();

  void InitializePIDControllers();
  void InitializeDashboard();
  void ReadDashboard();

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;

  double m_driveExponent = 1.2;
  bool intakeBool = false;
  bool uptakeBool;
  bool shootMan;
  bool limelightTrackingBool = false;


//So long, Joystick.h!
  frc::XboxController m_stick{0};
// A second controler
  frc::XboxController m_stick_climb{1};

  WPI_TalonFX m_frontRightMotor = {1};
  WPI_TalonFX m_backRightMotor = {2};
  WPI_TalonFX m_frontLeftMotor = {3};
  WPI_TalonFX m_backLeftMotor = {4};

  // Left side of the robot is inverted
  // Tonk drive
  frc::MotorControllerGroup m_leftDrive{m_frontLeftMotor, m_backLeftMotor};
  frc::MotorControllerGroup m_rightDrive{m_frontRightMotor, m_backRightMotor};

  frc::DifferentialDrive m_drive{m_leftDrive, m_rightDrive};

  //Neo motors
  static const int rotateIntakeMotorDeviceID = 5;
  static const int spinIntakeMotorDeviceID = 6;
// I don't know what either of theese do
  static const int shootingMotorAlphaDeviceID = 7;
  static const int shootingMotorBetaDeviceID = 8;
  static const int uptakeMotorDeviceID = 9;
  static const int rightClimberRotationNeoDeviceID = 10;
  static const int leftClimberRotationNeoDeviceID = 11;

// Climber falcons
  WPI_TalonFX m_leftClimberExtender = {12};
  WPI_TalonFX m_rightClimberExtender = {13};

  // REV bulldarn
rev::CANSparkMax m_rotateIntakeMotor{rotateIntakeMotorDeviceID, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax m_spinIntakeMotor{spinIntakeMotorDeviceID, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax m_shootingMotorAlpha{shootingMotorAlphaDeviceID, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax m_shootingMotorBeta{shootingMotorBetaDeviceID, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax m_uptakeMotor{uptakeMotorDeviceID, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax m_rightClimberRotationNeo{rightClimberRotationNeoDeviceID, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax m_leftClimberRotationNeo{leftClimberRotationNeoDeviceID, rev::CANSparkMax::MotorType::kBrushless};

//encoders
rev::SparkMaxRelativeEncoder m_rotateIntakeEncoder = m_rotateIntakeMotor.GetEncoder(); 
rev::SparkMaxRelativeEncoder m_spinIntakeEncoder = m_spinIntakeMotor.GetEncoder(); 
rev::SparkMaxRelativeEncoder m_shootingMotorAlphaEncoder = m_shootingMotorAlpha.GetEncoder(); 
rev::SparkMaxRelativeEncoder m_shootingMotorBetaEncoder = m_shootingMotorBeta.GetEncoder(); 
rev::SparkMaxRelativeEncoder m_rightClimberEncoder = m_rightClimberRotationNeo.GetEncoder(); 
rev::SparkMaxRelativeEncoder m_leftClimberEncoder = m_leftClimberRotationNeo.GetEncoder(); 


  //penumatics
  frc::DoubleSolenoid m_shooterShifter{frc::PneumaticsModuleType::REVPH, 1, 2};

//std::shared_ptr<NetworkTable> m_table = nt::NetworkTableInstance::GetDefault().GetTable("limelight-bepis"); 
double tx_OFFSET = 0.0;

//auto timer
frc::Timer autoTimer;

//PID Initialization -- have to manually set PIDs to motors each time??
frc2::PIDController m_rotateIntakePIDController{0.0, 0.0, 0.0}; //kP, kI, kD
frc2::PIDController m_leftClimberPIDController{0.0, 0.0, 0.0};
frc2::PIDController m_rightClimberPIDController{0.0, 0.0, 0.0};
frc2::PIDController m_shootingMotorAlphaPIDController{0.0, 0.0, 0.0};
frc2::PIDController m_shootingMotorBetaPIDController{0.0, 0.0, 0.0};


struct pidCoeff {
    double kP;
    double kI;
    double kD;
};

pidCoeff m_rotateIntakeCoeff{0.0, 0.0, 0.0};
pidCoeff m_leftClimberCoeff{0.0, 0.0, 0.0};
pidCoeff m_rightClimberCoeff{0.0, 0.0, 0.0};
pidCoeff m_shooterAlphaCoeff{0.0, 0.0, 0.0};
pidCoeff m_shooterBetaCoeff{0.0, 0.0, 0.0};


};
