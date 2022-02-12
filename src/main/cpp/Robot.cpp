// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <iostream>

#include <fmt/core.h>

#include <math.h>

#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {
  m_odometry = new frc::DifferentialDriveOdometry(frc::Rotation2d(0_deg));
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  m_chooser.AddOption(kThreeBallBlue, kThreeBallBlue);
  m_chooser.AddOption(kTwoBallBlue, kTwoBallBlue);
  m_chooser.AddOption(kThreeBallRed, kThreeBallRed);
  m_chooser.AddOption(kTwoBallRed, kTwoBallRed);

  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
// right side might need to be inverted depending on construction
  m_leftDrive.SetInverted(true);
  
  //  double ballsInShooter = 0; //add when break bar functionality is added
  shootMan = true;
  wrongBall = false;

  m_midRightMotor.Follow(m_frontRightMotor);
  m_midLeftMotor.Follow(m_frontLeftMotor);
  m_backRightMotor.Follow(m_frontRightMotor);
  m_backLeftMotor.Follow(m_frontLeftMotor);


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

  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  fmt::print("Auto selected: {}\n", m_autoSelected);

  if (m_autoSelected == kThreeBallBlue) {
    fs::path deployDirectory = frc::filesystem::GetDeployDirectory();
    deployDirectory = deployDirectory / "autos" / "Patheaver/autos/ThreeBallBlue.wpilib.json";
    m_trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());
  }

  if (m_autoSelected == kTwoBallBlue) {
    fs::path deployDirectory = frc::filesystem::GetDeployDirectory();
    deployDirectory = deployDirectory / "autos" / "Patheaver/autos/TwoBallBlue.wpilib.json";
    m_trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());
  }

  if (m_autoSelected == kThreeBallRed) { 
    fs::path deployDirectory = frc::filesystem::GetDeployDirectory();
    deployDirectory = deployDirectory / "autos" / "Patheaver/autos/ThreeBallRed.wpilib.json";
    m_trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());
  }

  if (m_autoSelected == kTwoBallRed) {
    fs::path deployDirectory = frc::filesystem::GetDeployDirectory();
    deployDirectory = deployDirectory / "autos" / "Patheaver/autos/TwoBallRed.wpilib.json";
    m_trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());
  }
      // Custom Auto goes here
  else {
      // Default Auto goes here
    }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }

  autoFollowPath();
  // Iteration one
  /*
   autoTimer.Start();
   if (autoTimer.Get() <= units::time::second_t(4)) {
     LimelightTracking();
     ShooterArm();
     ShooterFire();
   }
   if (autoTimer.Get() > units::time::second_t(4) && autoTimer.Get() <= units::time::second_t(5)) {
     m_drive.ArcadeDrive(0.5,0);
   }
*/

  // Iteration two
  /*
  autoTimer.Start();
  if (autoTimer.Get() <= units::time::second_t(0.5)) {
  }
  if (autoTimer.Get() > units::time::second_t(0.5) && autoTimer.Get() <= units::time::second_t(5)) {
    m_drive.ArcadeDrive(0.5, 0);
  }
  if  (autoTimer.Get() > units::time::second_t(5) && autoTimer.Get() <= units::time::second_t(8)) {
    m_drive.ArcadeDrive(0, 0.5);
  }
  if  (autoTimer.Get() > units::time::second_t(8) && autoTimer.Get() <= units::time::second_t(12)) {
    LimelightTracking();
  }
  if  (autoTimer.Get() > units::time::second_t(12) && autoTimer.Get() <= units::time::second_t(15)) {
    LimelightTracking();
  }
  */

  // Iteration three

  /*
  autoTimer.Start();
  if (autoTimer.Get() <= units::time::second_t(4)) {
    LimelightTracking();
    ShooterArm();
    ShooterFire();
  }
  if (autoTimer.Get() > units::time::second_t(4) && autoTimer.Get() <= units::time::second_t(8)) {

  }
  ready aim and fire the two extra balls
  */
}

void Robot::TeleopInit() {
  m_shooter.InitializePIDControllers();
}

void Robot::TeleopPeriodic() {

  //Read controller input

  double throttle = m_stick.GetRightTriggerAxis() - m_stick.GetLeftTriggerAxis();

  //Looks like Ethan wants exponents...
  double throttleExp = pow(throttle, m_driveExponent);
  double turnInput = pow(m_stick.GetRightX(), m_driveExponent);

  m_drive.ArcadeDrive(throttleExp, turnInput);



 //uptake
 if (m_stick.GetAButtonPressed()) {
   if (uptakeBool == true) {
     //stop uptake
     m_take.UptakeStop();
     uptakeBool = false;
   }
   if (uptakeBool == false) {
     //Start uptake
     m_take.UptakeStart(0.25);
     uptakeBool = true;
   }
 }


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
}
/* Shooter code that must be moved
//Fire!
void Robot::ShooterFire() {
  if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed){
    if (m_take.BallColor() == 'r') {wrongBall = false;}
    if (m_take.BallColor() == 'b') {wrongBall = true;}
    if (m_take.BallColor() == 'E') {std::cout << "[WARN]: Color Sensor issue \n";}
  }

  if (frc::DriverStation::GetAlliance()  == frc::DriverStation::Alliance::kBlue){
    if (m_take.BallColor() == 'r') {wrongBall = true;}
    if (m_take.BallColor() == 'b') {wrongBall = false;}
    if (m_take.BallColor() == 'E') {std::cout << "[WARN]: Color Sensor issue \n";}
}

if (wrongBall){
  //  sosTimer.Start()
  // Make an SOS
  m_stick.SetRumble(frc::GenericHID::RumbleType::kLeftRumble, 1.0);
  m_stick.SetRumble(frc::GenericHID::RumbleType::kRightRumble, 1.0);
 }
 if (!wrongBall){
   m_stick.SetRumble(frc::GenericHID::RumbleType::kLeftRumble, 0.0);
   m_stick.SetRumble(frc::GenericHID::RumbleType::kRightRumble, 0.0);
 }

  nt::NetworkTableEntry txEntry;
  nt::NetworkTableEntry tyEntry;
  nt::NetworkTableEntry taEntry;

  double ta, tx, ty;

  auto inst = nt::NetworkTableInstance::GetDefault();
  auto table = inst.GetTable("limelight-bepis");
  txEntry = table->GetEntry("tx");
  tyEntry = table->GetEntry("ty");
  taEntry = table->GetEntry("ta");


  txEntry.SetDouble(tx);
  tyEntry.SetDouble(ty);
  taEntry.SetDouble(ta);

  ty = ty * -1;

  if (shootMan){
    Robot::LimelightTracking();
    if (limelightTrackingBool == true) {
      //Code stolen. Procedure is to map ty to theta, subtract a value I forget and then you get your angle. Solve from there.

      double distance = ((heightOfTarget - heightLimelight) / tan((constantLimelightAngle + ty) * (3.141592653 / 180)));

      double rpm = CalculateRPM(distance);
      m_shooterAlphaPIDController.SetReference(rpm, rev::ControlType::kVelocity);
      m_shooterBetaPIDController.SetReference(rpm, rev::ControlType::kVelocity);
}
    else {
            if (tx < 0){
                        m_drive.ArcadeDrive(0, 0.5);
            }
            if (tx > 0){
                        m_drive.ArcadeDrive(0, -0.5);
            }
      }
  }

  if (!shootMan){
    //Code stolen. Procedure is to map ty to theta, subtract a value I forget and then you get your angle. Solve from there.
                 double distance = ((heightOfTarget - heightLimelight) / tan((constantLimelightAngle + ty) * (3.141592653 / 180)));

                 double rpm = CalculateRPM(distance);
                 m_shooterAlphaPIDController.SetReference(rpm, rev::ControlType::kVelocity);
                 m_shooterBetaPIDController.SetReference(rpm, rev::ControlType::kVelocity);
  }
}


double Robot::CalculateRPM(double d) {
  //Take real distance in feet and determine the needed RPMs
  //EXPERIMENTAL

  //double rpm = 0.0169 * d * d - 4.12 * d + 2614.5;
  //double rpm = 0.01474 * d * d - 3.573 * d + 2588.0;
  //double rpm = 0.0273 * d * d - 6.27 * d + 2901.3;
  double rpm = 0.0113 * d * d - 0.762 * d + 2290.1;
  return rpm;

}



// Ready!
void Robot::LimelightTracking() {
    nt::NetworkTableEntry txEntry;
    nt::NetworkTableEntry tyEntry;
    nt::NetworkTableEntry taEntry;

    double ta, tx, ty;

    auto inst = nt::NetworkTableInstance::GetDefault();
    auto table = inst.GetTable("limelight-bepis");
    txEntry = table->GetEntry("tx");
    tyEntry = table->GetEntry("ty");
    taEntry = table->GetEntry("ta");


    txEntry.SetDouble(tx);
    tyEntry.SetDouble(ty);
    taEntry.SetDouble(ta);

    std::cout << tx << ty << ta << "\n";

    
      // Dummy values
    if (ta <= taHighBound &&
        ta >= taLowBound &&
        tx <= txHighBound &&
        tx >= txLowBound &&
        ty <= tyHighBound &&
        ty >= tyLowBound)
      {
        limelightTrackingBool = true;
    }
    else {
        limelightTrackingBool = false;
    }

  //If it's tracking, use limebool
}
*/


void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::InitializePIDControllers() {
  //Climber intializes PIDs in it's own function
  m_climber.ClimberPIDInit();
  m_take.TakePIDInit();

}

void Robot::InitializeDashboard() {
  //Climbers do that in their own function
  m_climber.ClimberDashInit();
  m_take.TakeDashInit();

// Winch Motors

  /*
  if (shootMan){frc::SmartDashboard::PutNumber("Shooter Mode", "Auto");}
  if (!shootMan){frc::SmartDashboard::PutNumber("Shooter Mode", "Manual");}
  */

}

void Robot::ReadDashboard() {
  m_climber.ClimberDashRead();
  m_take.TakeDashRead();

  
}

void Robot::setSpeeds(const frc::DifferentialDriveWheelSpeeds& speeds) {
  const auto leftFeedforward = m_feedforward.Calculate(speeds.left);
  const auto rightFeedforward = m_feedforward.Calculate(speeds.right);
 
  const double leftOutput = m_frontRightMotorPIDController.Calculate(m_frontRightMotor.GetActiveTrajectoryVelocity(), speeds.left.to<double>());
  const double rightOutput = m_frontLeftMotorPIDController.Calculate(m_frontLeftMotor.GetActiveTrajectoryVelocity(), speeds.right.to<double>());
 
  m_leftGroup->SetVoltage(units::volt_t{leftOutput} + leftFeedforward);
  m_rightGroup->SetVoltage(units::volt_t{rightOutput} + rightFeedforward);
 
}

void Robot::autoDrive(units::meters_per_second_t xSpeed, units::radians_per_second_t rot){
  setSpeeds(m_kinematics.ToWheelSpeeds({xSpeed, 0_mps, rot}));
}

void Robot::autoFollowPath(){
  if (autoTimer.Get() < m_trajectory.TotalTime()) {
    auto desiredPose = m_trajectory.Sample(autoTimer.Get());
    auto refChassisSpeeds = controller1.Calculate(m_odometry->GetPose(), desiredPose);
 
    autoDrive(refChassisSpeeds.vx, refChassisSpeeds.omega);
  } else {
    autoDrive(0_mps, 0_rad_per_s);
  }
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
