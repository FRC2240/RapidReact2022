// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include "Climber.h"
#include "Take.h"

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
  //void ShooterAim();

  void ShooterArm();
  void ShooterFire();
  void IntakeDeploy();
  void IntakeReturn();

  void InitializePIDControllers();
  void InitializeDashboard();
  void ReadDashboard();


 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  const std::string kThreeBallBlueFirstBall = "ThreeBallBlueFirstBall";
  const std::string kThreeBallBlueSecondBall = "ThreeBallBlueSecondBall";
  const std::string kThreeBallBlueThirdBall = "ThreeBallBlueThirdBall";
  const std::string kTwoBallBlueFirstBall = "TwoBallBlueFirstBall";
  const std::string kTwoBallBlueSecondBall = "TwoBallBlueSecondBall";
  const std::string kThreeBallRedFirstBall = "ThreeBallRedFirstBall";
  const std::string kThreeBallRedSecondBall = "ThreeBallRedSecondBall";
  const std::string kThreeBallRedThirdBall = "ThreeBallRedThirdBall";
  const std::string kTwoBallRedFirstBall = "TwoBallRedFirstBall";
  const std::string kTwoBallRedSecondBall = "TwoBallRedSecondBall";
  std::string m_autoSelected;

  double m_driveExponent = 1.2;
  bool shootMan;
  bool limelightTrackingBool = false;
  fs::path deployDirectory;
  bool uptakeBool; //help me

  double taLowBound, taHighBound;
  double txLowBound, txHighBound;
  double tyLowBound, tyHighBound;
  double heightOfTarget;
  double heightLimelight;
  double constantLimelightAngle;
  Climber m_climber;
  Take m_take; 

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

  
// I don't know what either of theese do
  static const int shootingMotorAlphaDeviceID = 7;
  static const int shootingMotorBetaDeviceID = 8;
  static const int uptakeMotorDeviceID = 9;
  static const int uptakeIdleMotorDeviceID = 14;


  // REV bulldarn
  
rev::CANSparkMax m_shootingMotorAlpha{shootingMotorAlphaDeviceID, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax m_shootingMotorBeta{shootingMotorBetaDeviceID, rev::CANSparkMax::MotorType::kBrushless};


//encoders

rev::SparkMaxRelativeEncoder m_shootingMotorAlphaEncoder = m_shootingMotorAlpha.GetEncoder(); 
rev::SparkMaxRelativeEncoder m_shootingMotorBetaEncoder = m_shootingMotorBeta.GetEncoder(); 



//std::shared_ptr<NetworkTable> m_table = nt::NetworkTableInstance::GetDefault().GetTable("limelight-bepis");
double tx_OFFSET = 0.0;

//auto timer
frc::Timer autoTimer;

//PID Initialization -- have to manually set PIDs to motors each time??
rev::SparkMaxPIDController m_shooterAlphaPIDController = m_shootingMotorAlpha.GetPIDController();
rev::SparkMaxPIDController m_shooterBetaPIDController = m_shootingMotorBeta.GetPIDController();




struct pidCoeff {
    double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
};



  frc::Trajectory m_trajectory;



pidCoeff m_shooterAlphaCoeff{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
pidCoeff m_shooterBetaCoeff{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  

};
