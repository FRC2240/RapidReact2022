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

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
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
    // Use shooting function when it's a thing
  // autoTimer.Start();
  // if (autoTimer.Get() <= units::time::second_t(0.25)) {
  //   m_drive.ArcadeDrive(0.5, 0);
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
  if  (autoTimer.Get() > units::time::second_t(8) && autoTimer.Get() <= units::time::second_t(15)) {
    // Ready aim and fire twice
  }
  // Iteration three
    // shoot starting ball
    // pathfinder things 
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

 //Shoot the Mortar
 if (m_stick.GetRightBumperPressed()) {
   // Hold the winch at a certain point
   //TODO PIDs for that
   Robot::ShooterArm();
 }

 if (m_stick.GetStartButton()){
   shootMan = false;
 }

 if (m_stick.GetLeftBumperPressed()) {
   if (shootMan == true){
   Robot::ShooterFire();
   }
   else {
     m_shooterShifter.Set(frc::DoubleSolenoid::Value::kReverse); //possibly kForwards
   }
 }

}

// Ready!
void Robot::LimelightTracking() {
  //If it's tracking, use limebool
}

//Aim!
void Robot::ShooterArm() {
  //needs PIDs
}

//Fire!
void Robot::ShooterFire() {
  if (limelightTrackingBool == true) {
    m_shooterShifter.Set(frc::DoubleSolenoid::Value::kReverse); //possibly kForwards
  }
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
//rotate intake
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

//climber rotation
m_rightClimberRotatePIDController.SetP(m_rightClimberRotateCoeff.kP);
m_rightClimberRotatePIDController.SetI(m_rightClimberRotateCoeff.kI);
m_rightClimberRotatePIDController.SetD(m_rightClimberRotateCoeff.kD);
m_rightClimberRotatePIDController.SetIZone(m_rightClimberRotateCoeff.kIz);
m_rightClimberRotatePIDController.SetFF(m_rightClimberRotateCoeff.kFF);
m_rightClimberRotatePIDController.SetOutputRange(m_rightClimberRotateCoeff.kMinOutput, m_rightClimberRotateCoeff.kMaxOutput);

m_leftClimberRotatePIDController.SetP(m_leftClimberRotateCoeff.kP);
m_leftClimberRotatePIDController.SetI(m_leftClimberRotateCoeff.kI);
m_leftClimberRotatePIDController.SetD(m_leftClimberRotateCoeff.kD);
m_leftClimberRotatePIDController.SetIZone(m_leftClimberRotateCoeff.kIz);
m_leftClimberRotatePIDController.SetFF(m_leftClimberRotateCoeff.kFF);
m_leftClimberRotatePIDController.SetOutputRange(m_leftClimberRotateCoeff.kMinOutput, m_leftClimberRotateCoeff.kMaxOutput);

//falcons (climber extension)
m_rightClimberExtendPIDController.SetP(m_rightClimberExtendCoeff.kP);
m_rightClimberExtendPIDController.SetI(m_rightClimberExtendCoeff.kI);
m_rightClimberExtendPIDController.SetD(m_rightClimberExtendCoeff.kD);

m_leftClimberExtendPIDController.SetP(m_leftClimberExtendCoeff.kP);
m_leftClimberExtendPIDController.SetI(m_leftClimberExtendCoeff.kI);
m_leftClimberExtendPIDController.SetD(m_leftClimberExtendCoeff.kD);

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
  //falcons
  frc::SmartDashboard::PutNumber("Left Climber Extend P Gain", m_leftClimberExtendCoeff.kP);
  frc::SmartDashboard::PutNumber("Left Climber Extend I Gain", m_leftClimberExtendCoeff.kI);
  frc::SmartDashboard::PutNumber("Left Climber Extend D Gain", m_leftClimberExtendCoeff.kD);

  frc::SmartDashboard::PutNumber("Right Climber Extend P Gain", m_rightClimberExtendCoeff.kP);
  frc::SmartDashboard::PutNumber("Right Climber Extend I Gain", m_rightClimberExtendCoeff.kI);
  frc::SmartDashboard::PutNumber("Right Climber Extend D Gain", m_rightClimberExtendCoeff.kD);

  //rotate climber
  frc::SmartDashboard::PutNumber("Left Climber Rotate P Gain", m_leftClimberExtendCoeff.kP);
  frc::SmartDashboard::PutNumber("Left Climber Rotate I Gain", m_leftClimberExtendCoeff.kI);
  frc::SmartDashboard::PutNumber("Left Climber Rotate D Gain", m_leftClimberExtendCoeff.kD);
  frc::SmartDashboard::PutNumber("Left Climber Rotate Max Output", m_leftClimberExtendCoeff.kMaxOutput);
  frc::SmartDashboard::PutNumber("Left Climber Rotate Min Output", m_leftClimberExtendCoeff.kMinOutput);

  frc::SmartDashboard::PutNumber("Right Climber Rotate P Gain", m_rightClimberExtendCoeff.kP);
  frc::SmartDashboard::PutNumber("Right Climber Rotate I Gain", m_rightClimberExtendCoeff.kI);
  frc::SmartDashboard::PutNumber("Right Climber Rotate D Gain", m_rightClimberExtendCoeff.kD);
  frc::SmartDashboard::PutNumber("Left Climber Rotate Max Output", m_rightClimberExtendCoeff.kMaxOutput);
  frc::SmartDashboard::PutNumber("Left Climber Rotate Min Output", m_rightClimberExtendCoeff.kMinOutput);

//Rotate intake
  frc::SmartDashboard::PutNumber("Rotate Intake P Gain", m_rightClimberExtendCoeff.kP);
  frc::SmartDashboard::PutNumber("Rotate Intake I Gain", m_rightClimberExtendCoeff.kI);
  frc::SmartDashboard::PutNumber("Rotate Intake D Gain", m_rightClimberExtendCoeff.kD);
  frc::SmartDashboard::PutNumber("Rotate Intake Max Output", m_rightClimberExtendCoeff.kMaxOutput);
  frc::SmartDashboard::PutNumber("Rotate Intake Min Output", m_rightClimberExtendCoeff.kMinOutput);

 // Winch Motors
  frc::SmartDashboard::PutNumber("Alpha Motor P Gain", m_rightClimberExtendCoeff.kP);
  frc::SmartDashboard::PutNumber("Alpha Motor I Gain", m_rightClimberExtendCoeff.kI);
  frc::SmartDashboard::PutNumber("Alpha Motor D Gain", m_rightClimberExtendCoeff.kD);
  frc::SmartDashboard::PutNumber("Alpha Motor Max Output", m_rightClimberExtendCoeff.kMaxOutput);
  frc::SmartDashboard::PutNumber("Alpha Motor Min Output", m_rightClimberExtendCoeff.kMinOutput);

  frc::SmartDashboard::PutNumber("Beta Motor P Gain", m_rightClimberExtendCoeff.kP);
  frc::SmartDashboard::PutNumber("Beta Motor I Gain", m_rightClimberExtendCoeff.kI);
  frc::SmartDashboard::PutNumber("Beta Motor D Gain", m_rightClimberExtendCoeff.kD);
  frc::SmartDashboard::PutNumber("Beta Motor Max Output", m_rightClimberExtendCoeff.kMaxOutput);
  frc::SmartDashboard::PutNumber("Beta Motor Min Output", m_rightClimberExtendCoeff.kMinOutput);
}

void Robot::ReadDashboard() {
  double p, i, d, min, max;

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

// Climber Rotation
  p   = frc::SmartDashboard::GetNumber("Right Climber Rotate P Gain", 0);
  std::cout << "Read Dashboard Right Rotate Climber p gain: " << p << "\n";
  i   = frc::SmartDashboard::GetNumber("Right Climber Rotate I Gain", 0);
  std::cout << "Read Dashboard Right Climber Rotate i gain: " << i << "\n";
  d   = frc::SmartDashboard::GetNumber("Right Climber Rotate D Gain", 0);
  std::cout << "Read Dashboard Right Climber Rotate d gain: " << d << "\n";
  min = frc::SmartDashboard::GetNumber("Right Climber Rotate Min Output", 0);
  max = frc::SmartDashboard::GetNumber("Right Climber Rotate Max Output", 0);

  if ((p != m_rightClimberRotateCoeff.kP)) { m_rightClimberRotatePIDController.SetP(p);m_rightClimberRotateCoeff.kP = p; }
  if ((i != m_rightClimberRotateCoeff.kI)) { m_rightClimberRotatePIDController.SetI(i); m_rightClimberRotateCoeff.kI = i; }
  if ((d != m_rightClimberRotateCoeff.kD)) { m_rightClimberRotatePIDController.SetD(d); m_rightClimberRotateCoeff.kD = d; }
  if ((max != m_rightClimberRotateCoeff.kMaxOutput) || (min != m_rightClimberRotateCoeff.kMinOutput)) { 
    m_rightClimberRotatePIDController.SetOutputRange(min, max); 
    m_rightClimberRotateCoeff.kMinOutput = min; m_rotateIntakeCoeff.kMaxOutput = max; 
  }

   p   = frc::SmartDashboard::GetNumber("Left Climber Rotate P Gain", 0);
  std::cout << "Read Dashboard left Climber Rotate p gain: " << p << "\n";
  i   = frc::SmartDashboard::GetNumber("Left Climber Rotate I Gain", 0);
  std::cout << "Read Dashboard left Climber Rotate i gain: " << i << "\n";
  d   = frc::SmartDashboard::GetNumber("Left Climber Rotate D Gain", 0);
  std::cout << "Read Dashboard left Climber Rotate d gain: " << d << "\n";
  min = frc::SmartDashboard::GetNumber("Left Climber Rotate Min Output", 0);
  max = frc::SmartDashboard::GetNumber("Left Climber Rotate Max Output", 0);

  if ((p != m_leftClimberRotateCoeff.kP)) { m_leftClimberRotatePIDController.SetP(p);m_leftClimberRotateCoeff.kP = p; }
  if ((i != m_leftClimberRotateCoeff.kI)) { m_leftClimberRotatePIDController.SetI(i); m_leftClimberRotateCoeff.kI = i; }
  if ((d != m_leftClimberRotateCoeff.kD)) { m_leftClimberRotatePIDController.SetD(d); m_leftClimberRotateCoeff.kD = d; }
  if ((max != m_leftClimberRotateCoeff.kMaxOutput) || (min != m_leftClimberRotateCoeff.kMinOutput)) { 
    m_leftClimberRotatePIDController.SetOutputRange(min, max); 
    m_leftClimberRotateCoeff.kMinOutput = min; m_leftClimberRotateCoeff.kMaxOutput = max; 
  }

//Falcons (climber extension)
 p = frc::SmartDashboard::GetNumber("Right Climber Extend P Gain", 0);
  std::cout << "Read Dashboard Right Extend Climber p gain: " << p << "\n";
  i   = frc::SmartDashboard::GetNumber("Right Climber Extend I Gain", 0);
  std::cout << "Read Dashboard Right Climber Extend i gain: " << i << "\n";
  d   = frc::SmartDashboard::GetNumber("Right Climber Extend D Gain", 0);
  std::cout << "Read Dashboard Right Climber Extend d gain: " << d << "\n";

  if ((p != m_rightClimberExtendCoeff.kP)) { m_rightClimberExtendPIDController.SetP(p);m_rightClimberExtendCoeff.kP = p; }
  if ((i != m_rightClimberExtendCoeff.kI)) { m_rightClimberExtendPIDController.SetI(i); m_rightClimberExtendCoeff.kI = i; }
  if ((d != m_rightClimberExtendCoeff.kD)) { m_rightClimberExtendPIDController.SetD(d); m_rightClimberExtendCoeff.kD = d; }

   p   = frc::SmartDashboard::GetNumber("Left Climber Extend P Gain", 0);
  std::cout << "Read Dashboard left Climber Extend p gain: " << p << "\n";
  i   = frc::SmartDashboard::GetNumber("Left Climber Extend I Gain", 0);
  std::cout << "Read Dashboard left Climber Extend i gain: " << i << "\n";
  d   = frc::SmartDashboard::GetNumber("Left Climber Extend D Gain", 0);
  std::cout << "Read Dashboard left Climber Extend d gain: " << d << "\n";

  if ((p != m_leftClimberExtendCoeff.kP)) { m_leftClimberExtendPIDController.SetP(p);m_leftClimberExtendCoeff.kP = p; }
  if ((i != m_leftClimberExtendCoeff.kI)) { m_leftClimberExtendPIDController.SetI(i); m_leftClimberExtendCoeff.kI = i; }
  if ((d != m_leftClimberExtendCoeff.kD)) { m_leftClimberExtendPIDController.SetD(d); m_leftClimberExtendCoeff.kD = d; }

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
