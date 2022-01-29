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
  m_chooser.AddOption(kThreeBallBlue, kThreeBallBlue);
  m_chooser.AddOption(kTwoBallBlue,kTwoBallBlue);
  m_chooser.AddOption(kThreeBallRed,kThreeBallRed);
  m_chooser.AddOption(kTwoBallRed,kTwoBallRed);

  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
// right side might need to be inverted depending on construction
  m_leftDrive.SetInverted(true);
  //  double ballsInShooter = 0; //add when break bar functionality is added
  shootMan = true;

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
    deployDirectory = deployDirectory / "paths" / "Patheaver/Paths/ThreeBallBlue.wpilib.json";
    m_trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());
  }
  else if (m_autoSelected == kTwoBallBlue) {
    
    fs::path deployDirectory = frc::filesystem::GetDeployDirectory();
    deployDirectory = deployDirectory / "paths" / "Patheaver/Paths/TwoBallBlue.wpilib.json";
    m_trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());
  }
  else if (m_autoSelected == kThreeBallRed) {

    fs::path deployDirectory = frc::filesystem::GetDeployDirectory();
    deployDirectory = deployDirectory / "paths" / "Patheaver/Paths/ThreeBallRed.wpilib.json";
    m_trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());
  }
  else if (m_autoSelected == kTwoBallRed) {
    fs::path deployDirectory = frc::filesystem::GetDeployDirectory();
    deployDirectory = deployDirectory / "paths" / "Patheaver/Paths/TwoBallRed.wpilib.json";
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
  // autoTimer.Start();
  // if (autoTimer.Get() <= units::time::second_t(4)) {
  //   LimelightTracking();
  //   ShooterArm();
  //   ShooterFire();
  // }
  // if (autoTimer.Get() > units::time::second_t(4) && autoTimer.Get() <= units::time::second_t(5)) {
  //     m_drive.ArcadeDrive(0.5,0);
  // }

  // Iteration two
  autoTimer.Start();
  if (autoTimer.Get() <= units::time::second_t(0.5)) {
    IntakeDeploy();
  }
  if (autoTimer.Get() > units::time::second_t(0.5) && autoTimer.Get() <= units::time::second_t(5)) {
    m_uptakeMotor.Set(0.5);
    m_drive.ArcadeDrive(0.5, 0);
  }
  if  (autoTimer.Get() > units::time::second_t(5) && autoTimer.Get() <= units::time::second_t(8)) {
    IntakeReturn();
    m_uptakeMotor.Set(0);
    m_drive.ArcadeDrive(0, 0.5);
  }
  if  (autoTimer.Get() > units::time::second_t(8) && autoTimer.Get() <= units::time::second_t(12)) {
    LimelightTracking();
    m_uptakeMotor.Set(0.5);
  }
  if  (autoTimer.Get() > units::time::second_t(12) && autoTimer.Get() <= units::time::second_t(15)) {
    m_uptakeMotor.Set(0);
    LimelightTracking();
  }
  // Iteration three
  // autoTimer.Start();
  // if (autoTimer.Get() <= units::time::second_t(4)) {
  //   LimelightTracking();
  //   ShooterArm();
  //   ShooterFire();
  // }) && autoTimer.Get() <= units::time::second_t(8)) {

  // }
  // ready aim and fire the two extra balls
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {

  //Read controller input

double throttle = m_stick.GetRightTriggerAxis() - m_stick.GetLeftTriggerAxis();

//Looks like Ethan wants exponents...
double throttleExp = pow(throttle, m_driveExponent);
double turnInput = pow(m_stick.GetRightX(), m_driveExponent);

m_drive.ArcadeDrive(throttleExp, turnInput);


//Intake
 if (m_stick.GetYButtonPressed() == 1) {
   if (intakeBool == true) {
     // Is running, turn it off

     //TODO retract intake via PIDs here

    IntakeReturn();

     intakeBool = false;
}
   if (intakeBool == false) {
     // Not running, turn it on

     //TODO deploy intake
     IntakeDeploy();

     intakeBool = true;
   }
 }

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
      double distance = ((heightOfTarget - heightLimelight) / tan((constantLimelightAngle + ty) * (3.141592653 / 180)));

      double rpmA = ACalculateRPM(distance);
      m_shooterAlphaPIDController.SetReference(rpmA, rev::ControlType::kVelocity);
      double rpmB = BCalculateRPM(distance);
      m_shooterBetaPIDController.SetReference(rpmB, rev::ControlType::kVelocity);
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
                 double distance = ((heightOfTarget - heightLimelight) / tan((constantLimelightAngle + ty) * (3.141592653 / 180)));

                 double rpmA = ACalculateRPM(distance);
                 m_shooterAlphaPIDController.SetReference(rpmA, rev::ControlType::kVelocity);
                 double rpmB = BCalculateRPM(distance);
                 m_shooterBetaPIDController.SetReference(rpmB, rev::ControlType::kVelocity);
  }
}


double Robot::ACalculateRPM(double ad) {
  //double rpm = 0.0169 * d * d - 4.12 * d + 2614.5;
  //double rpm = 0.01474 * d * d - 3.573 * d + 2588.0;
  //double rpm = 0.0273 * d * d - 6.27 * d + 2901.3;
  double rpm = 0.0113 * ad * ad - 0.762 * ad + 2290.1;
  return rpm;
}
double Robot::BCalculateRPM(double bd) {
  //double rpm = 0.0169 * d * d - 4.12 * d + 2614.5;
  //double rpm = 0.01474 * d * d - 3.573 * d + 2588.0;
  //double rpm = 0.0273 * d * d - 6.27 * d + 2901.3;
  double rpm = 0.0113 * bd * bd - 0.762 * bd + 2290.1;
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





void Robot::IntakeDeploy() {
  double setpoint;
  m_rotateIntakePIDController.SetReference(setpoint, rev::ControlType::kSmartMotion);
  m_spinIntakeMotor.Set(-0.5);
}

void Robot::IntakeReturn(){
  m_spinIntakeMotor.Set(0.0);
  m_rotateIntakePIDController.SetReference(0.0, rev::ControlType::kSmartMotion);;
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::InitializePIDControllers() {
  //Climber intializes PIDs in it's own function
  m_climber.ClimberPIDInit();

  m_rotateIntakePIDController.SetP(m_rotateIntakeCoeff.kP);
  m_rotateIntakePIDController.SetI(m_rotateIntakeCoeff.kI);
  m_rotateIntakePIDController.SetD(m_rotateIntakeCoeff.kD);
  m_rotateIntakePIDController.SetIZone(m_rotateIntakeCoeff.kIz);
  m_rotateIntakePIDController.SetFF(m_rotateIntakeCoeff.kFF);
  m_rotateIntakePIDController.SetOutputRange(m_rotateIntakeCoeff.kMinOutput, m_rotateIntakeCoeff.kMaxOutput);

  m_rotateIntakePIDController.SetP(m_rotateIntakeCoeff.kP);
  m_rotateIntakePIDController.SetI(m_rotateIntakeCoeff.kI);
  m_rotateIntakePIDController.SetD(m_rotateIntakeCoeff.kD);
  m_rotateIntakePIDController.SetIZone(m_rotateIntakeCoeff.kIz);
  m_rotateIntakePIDController.SetFF(m_rotateIntakeCoeff.kFF);
  m_rotateIntakePIDController.SetOutputRange(m_rotateIntakeCoeff.kMinOutput, m_rotateIntakeCoeff.kMaxOutput);

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

//Rotate intake
  frc::SmartDashboard::PutNumber("Rotate Intake P Gain", m_rotateIntakeCoeff.kP);
  frc::SmartDashboard::PutNumber("Rotate Intake I Gain", m_rotateIntakeCoeff.kI);
  frc::SmartDashboard::PutNumber("Rotate Intake D Gain", m_rotateIntakeCoeff.kD);
  frc::SmartDashboard::PutNumber("Rotate Intake Max Output", m_rotateIntakeCoeff.kMaxOutput);
  frc::SmartDashboard::PutNumber("Rotate Intake Min Output", m_rotateIntakeCoeff.kMinOutput);

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

  //rotate intake
  // read PID coefficients from SmartDashboard
  p   = frc::SmartDashboard::GetNumber("Rotate Intake P Gain", 0);
  std::cout << "Read Dashboard rotate intake p gain: " << p << "\n";
  i   = frc::SmartDashboard::GetNumber("Rotate Intake I Gain", 0);
  std::cout << "Read Dashboard rotate intake i gain: " << i << "\n";
  d   = frc::SmartDashboard::GetNumber("Rotate Intake D Gain", 0);
  std::cout << "Read Dashboard rotate intake d gain: " << d << "\n";
  min = frc::SmartDashboard::GetNumber("Rotate Intake Min Output", 0);
  max = frc::SmartDashboard::GetNumber("Rotate Intake Max Output", 0);


  // If PID coefficients on SmartDashboard have changed, write new values to controller
  if ((p != m_rotateIntakeCoeff.kP)) { m_rotateIntakePIDController.SetP(p); m_rotateIntakeCoeff.kP = p; }
  if ((i != m_rotateIntakeCoeff.kI)) { m_rotateIntakePIDController.SetI(i); m_rotateIntakeCoeff.kI = i; }
  if ((d != m_rotateIntakeCoeff.kD)) { m_rotateIntakePIDController.SetD(d); m_rotateIntakeCoeff.kD = d; }
  if ((max != m_rotateIntakeCoeff.kMaxOutput) || (min != m_rotateIntakeCoeff.kMinOutput)) { 
    m_rotateIntakePIDController.SetOutputRange(min, max); 
    m_rotateIntakeCoeff.kMinOutput = min; m_rotateIntakeCoeff.kMaxOutput = max; 
  }


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
