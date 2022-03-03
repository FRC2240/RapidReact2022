// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Drivetrain.h"

void Drivetrain::SetSpeeds(const frc::DifferentialDriveWheelSpeeds& speeds) {
  const auto leftFeedforward = m_feedforward.Calculate(speeds.left);
  const auto rightFeedforward = m_feedforward.Calculate(speeds.right);
  //std::cout << "speeds: " << speeds.right << " " << speeds.left << std::endl;
  //std::cout << "feeds: " << rightFeedforward << " " << leftFeedforward << std::endl;
  //std::cout << "vel: " << m_rightEncoder->GetVelocity() << " " << -m_leftEncoder->GetVelocity() << std::endl;

  const double leftOutput = m_leftPIDController.Calculate(
      -m_leftMotor->GetSelectedSensorVelocity(), speeds.left.to<double>());
  const double rightOutput = m_rightPIDController.Calculate(
      m_rightMotor->GetSelectedSensorVelocity(), speeds.right.to<double>());

  //std::cout << "outputs: " << rightOutput << " " << leftOutput << std::endl;
  //std::cout << "voltages: " << units::volt_t{rightOutput} + rightFeedforward <<
  // " " << units::volt_t{leftOutput} + leftFeedforward << std::endl;

  m_leftGroup->SetVoltage(units::volt_t{leftOutput} + leftFeedforward);
  m_rightGroup->SetVoltage(units::volt_t{rightOutput} + rightFeedforward);
}

void Drivetrain::Drive(units::meters_per_second_t xSpeed,
                       units::radians_per_second_t rot) {
  //std::cout << "drive speed: " << xSpeed << " drive rot:" << rot << std::endl;
  SetSpeeds(m_kinematics.ToWheelSpeeds({xSpeed, 0_mps, rot}));
}

void Drivetrain::UpdateOdometry() {
  //std::cout << "update pos: " << m_gyro->GetRotation2d().Degrees() << " "  << m_rightEncoder->GetPosition() << " " << -m_leftEncoder->GetPosition() << std::endl;
  m_odometry->Update(m_gyro->GetRotation2d(),
                     units::meter_t(-m_leftMotor->GetSelectedSensorPosition()),
                     units::meter_t(m_rightMotor->GetSelectedSensorPosition()));
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