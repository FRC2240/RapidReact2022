// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include "rev/CANSparkMax.h"
#include "ctre/Phoenix.h"

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
// void InitializeDashboard();
// void InitializePIDControllers();
// void ReadDashboard();

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;

//So long, Joystick.h!
  frc::XboxController m_stick{0};
// A second controler 
  frc::XboxController m_stick_climb{1};\

  //Neo motors
  static const int leftIntakeMotorDeviceID = 5;
  static const int rightIntakeMotorDeviceID = 6;
// I don't know what either of theese do
  static const int rotationControlMotorDeviceID = 7;
  static const int spinningMotorDeviceID = 8;
  
rev::CANSparkMax m_leftIntakeMotor{leftIntakeMotorDeviceID, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax m_rightIntakeMotor{rightIntakeMotorDeviceID, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax m_rotationControlMotor{rotationControlMotorDeviceID, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax m_spinningMotorDeviceID{spinningMotorDeviceID, rev::CANSparkMax::MotorType::kBrushless};



  WPI_TalonFX m_frontRightMotor = {1};
  WPI_TalonFX m_backRightMotor = {2};
  WPI_TalonFX m_frontLeftMotor = {3}; 
  WPI_TalonFX m_backLeftMotor = {4}; 

// Left side of the robot might need to be inverted
// Tonk drive
  frc::MotorControllerGroup m_leftDrive{m_frontLeftMotor, m_backLeftMotor};
  frc::MotorControllerGroup m_rightDrive{m_frontRightMotor, m_backRightMotor};


  frc::DifferentialDrive m_drive{m_leftDrive, m_rightDrive};


// A nice arrangment of motors

};
