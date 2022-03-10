// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Drivetrain.h"

void Drivetrain::SetSpeeds(const frc::DifferentialDriveWheelSpeeds& speeds) {
  const auto leftFeedforward = m_feedforward.Calculate(speeds.left);
  const auto rightFeedforward = m_feedforward.Calculate(speeds.right);

  const double leftOutput = m_leftPIDController.Calculate(
      -m_leftMotor->GetSelectedSensorVelocity()*kFalconVelocitytoMPS, speeds.left.to<double>());
  const double rightOutput = m_rightPIDController.Calculate(
      m_rightMotor->GetSelectedSensorVelocity()*kFalconVelocitytoMPS, speeds.right.to<double>());


  m_leftGroup->SetVoltage(units::volt_t{leftOutput} + leftFeedforward);
  m_rightGroup->SetVoltage(units::volt_t{rightOutput} + rightFeedforward);
}

void Drivetrain::Drive(units::meters_per_second_t xSpeed,
                       units::radians_per_second_t rot) {
  SetSpeeds(m_kinematics.ToWheelSpeeds({xSpeed, 0_mps, rot}));
}

void Drivetrain::UpdateOdometry() {
  m_odometry->Update(m_gyro->GetRotation2d(),
                     units::meter_t(-m_leftMotor->GetSelectedSensorPosition()*(kDistancePerEncoderRotation/2048.0)),
                     units::meter_t(m_rightMotor->GetSelectedSensorPosition()*(kDistancePerEncoderRotation/2048.0)));
}

void Drivetrain::ResetOdometry(const frc::Pose2d& pose) {
  m_odometry->ResetPosition(pose, m_gyro->GetRotation2d());
}

frc::Pose2d Drivetrain::GetPose() const {
  return m_odometry->GetPose();
}

units::angle::degree_t Drivetrain::GetRotation() {
  return m_gyro->GetRotation2d().Degrees();
}
