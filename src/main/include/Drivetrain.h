// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <iostream>
#include <frc/DriverStation.h>
#include <frc/Encoder.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <units/angular_velocity.h>
#include <units/acceleration.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>

#include "ctre/Phoenix.h"
#include "AHRS.h"

/**
 * Represents a differential drive style drivetrain.
 */
class Drivetrain {
 public:
  Drivetrain(
      frc::MotorControllerGroup* leftGroup,
      frc::MotorControllerGroup* rightGroup,
      WPI_TalonFX* leftMotor,
      WPI_TalonFX* rightMotor
      ) :
  m_leftGroup(leftGroup),
  m_rightGroup(rightGroup),
  m_leftMotor(leftMotor),
  m_rightMotor(rightMotor)
  {
    // Set the distance per pulse for the drive encoders. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.

    m_leftMotor->SetSelectedSensorPosition(0.0);
    m_rightMotor->SetSelectedSensorPosition(0.0);

    m_leftGroup->SetInverted(true);

    // Instantiate gyro
    try {
			m_gyro = new AHRS(frc::SPI::Port::kMXP);
		} catch (std::exception& ex ) {
			std::string err_string = "Error instantiating navX MXP:  ";
			err_string += ex.what();
            std::cout << err_string << std::endl;
		}
    
    m_odometry = new frc::DifferentialDriveOdometry(m_gyro->GetRotation2d());
  }

  void SetSpeeds(const frc::DifferentialDriveWheelSpeeds& speeds);
  void Drive(units::meters_per_second_t xSpeed,
             units::radians_per_second_t rot);
  void UpdateOdometry();
  void ResetOdometry(const frc::Pose2d& pose);
  frc::Pose2d GetPose() const;
  units::angle::degree_t GetRotation();

  void ResetEncoders() {
    m_leftMotor->SetSelectedSensorPosition(0.0);
    m_rightMotor->SetSelectedSensorPosition(0.0);
  };

  private:

  static constexpr double kP = 1.685; //0.157; //2.77;                  // measured
  static constexpr auto   kS = 0.620_V; //0.27_V;                         // measured
  static constexpr auto   kV = 1.429 * 1_V * 1_s / 1_m;    // 1.53     // measured
  static constexpr auto   kA = 0.117 * 1_V * 1_s * 1_s / 1_m;  // 0.254 // measured

  units::meter_t kTrackWidth = 0.593_m; //0.657_m;                        // measured    
  double kDistancePerEncoderRotation = 0.0373; //((3.142*6.25/13.5)*0.0254); //0.0387;     // measured (meters)  
  double kFalconVelocityToRPM = (600.0/2048.0);
  double kFalconVelocitytoMPS = (kFalconVelocityToRPM*kDistancePerEncoderRotation/60.0);

  frc::MotorControllerGroup* m_leftGroup;
  frc::MotorControllerGroup* m_rightGroup;

  WPI_TalonFX* m_leftMotor;
  WPI_TalonFX* m_rightMotor;

  frc2::PIDController m_leftPIDController{kP, 0.0, 0.0};
  frc2::PIDController m_rightPIDController{kP, 0.0, 0.0};

  AHRS* m_gyro;

  frc::DifferentialDriveKinematics m_kinematics{kTrackWidth};
  frc::DifferentialDriveOdometry* m_odometry;

  // Gains are for example purposes only - must be determined for your own
  // robot!
  frc::SimpleMotorFeedforward<units::meters> m_feedforward{kS, kV, kA};

};
