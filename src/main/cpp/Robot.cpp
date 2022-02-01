// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <iostream>

#include <fmt/core.h>

#include <math.h>

#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  m_chooser.AddOption(kThreeBallBlueFirstBall, kThreeBallBlueFirstBall);
  m_chooser.AddOption(kThreeBallBlueSecondBall, kThreeBallBlueSecondBall);
  m_chooser.AddOption(kThreeBallBlueThirdBall, kThreeBallBlueThirdBall);
  m_chooser.AddOption(kTwoBallBlueFirstBall,kTwoBallBlueFirstBall);
  m_chooser.AddOption(kTwoBallBlueSecondBall,kTwoBallBlueSecondBall);
  m_chooser.AddOption(kThreeBallRedFirstBall,kThreeBallRedFirstBall);
  m_chooser.AddOption(kThreeBallRedSecondBall,kThreeBallRedSecondBall);
  m_chooser.AddOption(kThreeBallRedThirdBall,kThreeBallRedThirdBall);
  m_chooser.AddOption(kTwoBallRedFirstBall,kTwoBallRedFirstBall);
  m_chooser.AddOption(kTwoBallRedSecondBall,kTwoBallRedSecondBall);

  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
// right side might need to be inverted depending on construction
  m_leftDrive.SetInverted(true);
  //  double ballsInShooter = 0; //add when break bar functionality is added
  shootMan = true;
  wrongBall = false;

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

  if (m_autoSelected == kThreeBallBlueFirstBall) {
      
    fs::path deployDirectory = frc::filesystem::GetDeployDirectory();
    deployDirectory = deployDirectory / "paths" / "Patheaver/Paths/ThreeBallBlueFirstBall.wpilib.json";
    m_trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());
  }
  else if (m_autoSelected == kThreeBallBlueSecondBall) {
      
    fs::path deployDirectory = frc::filesystem::GetDeployDirectory();
    deployDirectory = deployDirectory / "paths" / "Patheaver/Paths/ThreeBallBlueSecondBall.wpilib.json";
    m_trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());
  }
  else if (m_autoSelected == kThreeBallBlueThirdBall) {
      
    fs::path deployDirectory = frc::filesystem::GetDeployDirectory();
    deployDirectory = deployDirectory / "paths" / "Patheaver/Paths/ThreeBallBlueThirdBall.wpilib.json";
    m_trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());
  }
  else if (m_autoSelected == kTwoBallBlueFirstBall) {
    
    fs::path deployDirectory = frc::filesystem::GetDeployDirectory();
    deployDirectory = deployDirectory / "paths" / "Patheaver/Paths/TwoBallBlueFirstBall.wpilib.json";
    m_trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());
  }
  else if (m_autoSelected == kTwoBallBlueSecondBall) {
    
    fs::path deployDirectory = frc::filesystem::GetDeployDirectory();
    deployDirectory = deployDirectory / "paths" / "Patheaver/Paths/TwoBallBlueSecondBall.wpilib.json";
    m_trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());
  }
  else if (m_autoSelected == kThreeBallRedFirstBall) {

    fs::path deployDirectory = frc::filesystem::GetDeployDirectory();
    deployDirectory = deployDirectory / "paths" / "Patheaver/Paths/ThreeBallRedFirstBall.wpilib.json";
    m_trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());
  }
  else if (m_autoSelected == kThreeBallRedSecondBall) {

    fs::path deployDirectory = frc::filesystem::GetDeployDirectory();
    deployDirectory = deployDirectory / "paths" / "Patheaver/Paths/ThreeBallRedSecondBall.wpilib.json";
    m_trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());
  }
  else if (m_autoSelected == kThreeBallRedThirdBall) {

    fs::path deployDirectory = frc::filesystem::GetDeployDirectory();
    deployDirectory = deployDirectory / "paths" / "Patheaver/Paths/ThreeBallRedThirdBall.wpilib.json";
    m_trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());
  }
  else if (m_autoSelected == kTwoBallRedFirstBall) {
    fs::path deployDirectory = frc::filesystem::GetDeployDirectory();
    deployDirectory = deployDirectory / "paths" / "Patheaver/Paths/TwoBallRedFirstBall.wpilib.json";
    m_trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());
  }
  else if (m_autoSelected == kTwoBallRedSecondBall) {
    fs::path deployDirectory = frc::filesystem::GetDeployDirectory();
    deployDirectory = deployDirectory / "paths" / "Patheaver/Paths/TwoBallRedSecondBall.wpilib.json";
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

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {

  //Read controller input

double throttle = m_stick.GetRightTriggerAxis() - m_stick.GetLeftTriggerAxis();

//Looks like Ethan wants exponents...
double throttleExp = pow(throttle, m_driveExponent);
double turnInput = pow(m_stick.GetRightX(), m_driveExponent);

m_drive.ArcadeDrive(throttleExp, turnInput);

/* why does this still exist?
//Intake
 

 //uptake
 if (m_stick.GetAButtonPressed()) {
   if (uptakeBool == true) {
     //stop uptake
     m_uptakeMotor.Set(0.0);
     uptakeBool = false;
   }
   if (uptakeBool == false) {
     //Start uptake
     m_uptakeMotor.Set(0.5);
     uptakeBool = true;
   }
 }

*/

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
   Robot::ShooterFire();
}
}
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


void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::InitializePIDControllers() {
  //Climber intializes PIDs in it's own function
  m_climber.ClimberPIDInit();
  m_take.TakePIDInit();
  
  //winch motors
m_shooterAlphaPIDController.SetP(m_shooterAlphaCoeff.kP);
m_shooterAlphaPIDController.SetI(m_shooterAlphaCoeff.kI);
m_shooterAlphaPIDController.SetD(m_shooterAlphaCoeff.kD);
m_shooterAlphaPIDController.SetIZone(m_shooterAlphaCoeff.kIz);
m_shooterAlphaPIDController.SetFF(m_shooterAlphaCoeff.kFF);
m_shooterAlphaPIDController.SetOutputRange(m_shooterAlphaCoeff.kMinOutput, m_shooterAlphaCoeff.kMaxOutput);

m_shooterBetaPIDController.SetP(m_shooterBetaCoeff.kP);
m_shooterBetaPIDController.SetI(m_shooterBetaCoeff.kI);
m_shooterBetaPIDController.SetD(m_shooterBetaCoeff.kD);
m_shooterBetaPIDController.SetIZone(m_shooterBetaCoeff.kIz);
m_shooterBetaPIDController.SetFF(m_shooterBetaCoeff.kFF);
m_shooterBetaPIDController.SetOutputRange(m_shooterBetaCoeff.kMinOutput, m_shooterBetaCoeff.kMaxOutput);

}

void Robot::InitializeDashboard() {
  //Climbers do that in their own function
  m_climber.ClimberDashInit();
  m_take.TakeDashInit();

// Winch Motors
  frc::SmartDashboard::PutNumber("Alpha Motor P Gain", m_shooterAlphaCoeff.kP);
  frc::SmartDashboard::PutNumber("Alpha Motor I Gain", m_shooterAlphaCoeff.kI);
  frc::SmartDashboard::PutNumber("Alpha Motor D Gain", m_shooterAlphaCoeff.kD);
  frc::SmartDashboard::PutNumber("Alpha Motor Max Output", m_shooterAlphaCoeff.kMaxOutput);
  frc::SmartDashboard::PutNumber("Alpha Motor Min Output", m_shooterAlphaCoeff.kMinOutput);

  frc::SmartDashboard::PutNumber("Beta Motor P Gain", m_shooterBetaCoeff.kP);
  frc::SmartDashboard::PutNumber("Beta Motor I Gain", m_shooterBetaCoeff.kI);
  frc::SmartDashboard::PutNumber("Beta Motor D Gain", m_shooterBetaCoeff.kD);
  frc::SmartDashboard::PutNumber("Beta Motor Max Output", m_shooterBetaCoeff.kMaxOutput);
  frc::SmartDashboard::PutNumber("Beta Motor Min Output", m_shooterBetaCoeff.kMinOutput);

  /*
  if (shootMan){frc::SmartDashboard::PutNumber("Shooter Mode", "Auto");}
  if (!shootMan){frc::SmartDashboard::PutNumber("Shooter Mode", "Manual");}
  */

}

void Robot::ReadDashboard() {
  double p, i, d, min, max;
  m_climber.ClimberDashRead();
  m_take.TakeDashRead();

  // Shooting motors

  p   = frc::SmartDashboard::GetNumber("Alpha Motor P Gain", 0);
  std::cout << "Read Dashboard Alpha Motor P Gain: " << p << "\n";
  i   = frc::SmartDashboard::GetNumber("Alpha Motor I Gain", 0);
  std::cout << "Read Dashboard Alpha Motor I Gain: " << i << "\n";
  d   = frc::SmartDashboard::GetNumber("Alpha Motor D Gain", 0);
  std::cout << "Read Dashboard Alpha Motor D Gain: " << d << "\n";
  min = frc::SmartDashboard::GetNumber("Alpha Motor Min Output", 0);
  max = frc::SmartDashboard::GetNumber("Alpha Motor Max Output", 0);

  if ((p != m_shooterAlphaCoeff.kP)) { m_shooterAlphaPIDController.SetP(p);m_shooterAlphaCoeff.kP = p; }
  if ((i != m_shooterAlphaCoeff.kI)) { m_shooterAlphaPIDController.SetI(i); m_shooterAlphaCoeff.kI = i; }
  if ((d != m_shooterAlphaCoeff.kD)) { m_shooterAlphaPIDController.SetD(d); m_shooterAlphaCoeff.kD = d; }
  if ((max != m_shooterAlphaCoeff.kMaxOutput) || (min != m_shooterAlphaCoeff.kMinOutput)) { 
    m_shooterAlphaPIDController.SetOutputRange(min, max); 
    m_shooterAlphaCoeff.kMinOutput = min; m_shooterAlphaCoeff.kMaxOutput = max; 
  }

   p   = frc::SmartDashboard::GetNumber("Beta Motor P Gain", 0);
  std::cout << "Read Dashboard Beta Motor P Gain: " << p << "\n";
  i   = frc::SmartDashboard::GetNumber("Beta Motor I Gain", 0);
  std::cout << "Read Dashboard Beta Motor I Gain: " << i << "\n";
  d   = frc::SmartDashboard::GetNumber("Beta Motor D Gain", 0);
  std::cout << "Read Dashboard Beta Motor D Gain: " << d << "\n";
  min = frc::SmartDashboard::GetNumber("Beta Motor Min Output", 0);
  max = frc::SmartDashboard::GetNumber("Beta Motor Max Output", 0);

  if ((p != m_shooterBetaCoeff.kP)) { m_shooterBetaPIDController.SetP(p);m_shooterBetaCoeff.kP = p; }
  if ((i != m_shooterBetaCoeff.kI)) { m_shooterBetaPIDController.SetI(i); m_shooterBetaCoeff.kI = i; }
  if ((d != m_shooterBetaCoeff.kD)) { m_shooterBetaPIDController.SetD(d); m_shooterBetaCoeff.kD = d; }
  if ((max != m_shooterBetaCoeff.kMaxOutput) || (min != m_shooterBetaCoeff.kMinOutput)) { 
    m_shooterBetaPIDController.SetOutputRange(min, max); 
    m_shooterBetaCoeff.kMinOutput = min; m_shooterBetaCoeff.kMaxOutput = max; 
  }
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
