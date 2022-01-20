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
  //   m_drive.ArcadeDrive(-.5, 0);
  // }

  // Iteration two
  autoTimer.Start();
  if (autoTimer.Get() <= units::time::second_t(5)) {
    // Lower intake and turn it on
    m_drive.ArcadeDrive(.5, 0);
  }
  if  (autoTimer.Get() > units::time::second_t(5) && autoTimer.Get() <= units::time::second_t(8)) {
    // turn off intake
    m_drive.ArcadeDrive(0, .5);
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
     m_spinIntakeMotor.Set(0);

     //TODO retract intake via PIDs here
     // experimenting idk how to actually set these yet
     double setpoint; 
     m_rotateIntakeMotor.Set(m_rotateIntakePIDController.Calculate(m_rotateIntakeEncoder.GetPosition(), setpoint)); 
     

     intakeBool = false;
}
   if (intakeBool == false) {
     // Not running, turn it on

     //TODO deploy intake

     m_spinIntakeMotor.Set(-0.5);
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

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::InitializePIDControllers() {
  m_rotateIntakePIDController.SetP(m_rotateIntakeCoeff.kP);
  m_rotateIntakePIDController.SetI(m_rotateIntakeCoeff.kI);
  m_rotateIntakePIDController.SetD(m_rotateIntakeCoeff.kD);

  m_rightClimberPIDController.SetP(m_rightClimberCoeff.kP);
  m_rightClimberPIDController.SetI(m_rightClimberCoeff.kI);
  m_rightClimberPIDController.SetD(m_rightClimberCoeff.kD);

  m_leftClimberPIDController.SetP(m_leftClimberCoeff.kP);
  m_leftClimberPIDController.SetI(m_leftClimberCoeff.kI);
  m_leftClimberPIDController.SetD(m_leftClimberCoeff.kD);
}

void Robot::InitializeDashboard() {
  frc::SmartDashboard::PutNumber("Rotate Intake P Gain", m_rotateIntakeCoeff.kP);
  frc::SmartDashboard::PutNumber("Rotate Intake I Gain", m_rotateIntakeCoeff.kI);
  frc::SmartDashboard::PutNumber("Rotate Intake D Gain", m_rotateIntakeCoeff.kD);

  frc::SmartDashboard::PutNumber("Right Climber P Gain", m_rightClimberCoeff.kP);
  frc::SmartDashboard::PutNumber("Right Climber I Gain", m_rightClimberCoeff.kI);
  frc::SmartDashboard::PutNumber("Right Climber D Gain", m_rightClimberCoeff.kD);

  frc::SmartDashboard::PutNumber("Left Climber P Gain", m_leftClimberCoeff.kP);
  frc::SmartDashboard::PutNumber("Left Climber I Gain", m_leftClimberCoeff.kI);
  frc::SmartDashboard::PutNumber("Left Climber D Gain", m_leftClimberCoeff.kD);
}

void Robot::ReadDashboard() {
  double p, i, d;
  // read PID coefficients from SmartDashboard
  p   = frc::SmartDashboard::GetNumber("Rotate Intake P Gain", 0);
  std::cout << "Read Dashboard rotate intake p gain: " << p << "\n";
  i   = frc::SmartDashboard::GetNumber("Rotate Intake I Gain", 0);
  std::cout << "Read Dashboard rotate intake i gain: " << i << "\n";
  d   = frc::SmartDashboard::GetNumber("Rotate Intake D Gain", 0);
  std::cout << "Read Dashboard rotate intake d gain: " << d << "\n";

  // If PID coefficients on SmartDashboard have changed, write new values to controller
  if ((p != m_rotateIntakeCoeff.kP)) { m_rotateIntakePIDController.SetP(p); m_rotateIntakeCoeff.kP = p; }
  if ((i != m_rotateIntakeCoeff.kI)) { m_rotateIntakePIDController.SetI(i); m_rotateIntakeCoeff.kI = i; }
  if ((d != m_rotateIntakeCoeff.kD)) { m_rotateIntakePIDController.SetD(d); m_rotateIntakeCoeff.kD = d; }


  p   = frc::SmartDashboard::GetNumber("Right Climber P Gain", 0);
  std::cout << "Read Dashboard Right Climber p gain: " << p << "\n";
  i   = frc::SmartDashboard::GetNumber("Right Climber I Gain", 0);
  std::cout << "Read Dashboard Right Climber i gain: " << i << "\n";
  d   = frc::SmartDashboard::GetNumber("Right Climber D Gain", 0);
  std::cout << "Read Dashboard Right Climber d gain: " << d << "\n";

  // If PID coefficients on SmartDashboard have changed, write new values to controller
  if ((p != m_rightClimberCoeff.kP)) { m_rightClimberPIDController.SetP(p);m_rightClimberCoeff.kP = p; }
  if ((i != m_rightClimberCoeff.kI)) { m_rightClimberPIDController.SetI(i); m_rightClimberCoeff.kI = i; }
  if ((d != m_rightClimberCoeff.kD)) { m_rightClimberPIDController.SetD(d); m_rightClimberCoeff.kD = d; }

   p   = frc::SmartDashboard::GetNumber("Left Climber P Gain", 0);
  std::cout << "Read Dashboard left Climber p gain: " << p << "\n";
  i   = frc::SmartDashboard::GetNumber("Left Climber I Gain", 0);
  std::cout << "Read Dashboard left Climber i gain: " << i << "\n";
  d   = frc::SmartDashboard::GetNumber("Left Climber D Gain", 0);
  std::cout << "Read Dashboard left Climber d gain: " << d << "\n";

  // If PID coefficients on SmartDashboard have changed, write new values to controller
  if ((p != m_leftClimberCoeff.kP)) { m_leftClimberPIDController.SetP(p);m_leftClimberCoeff.kP = p; }
  if ((i != m_leftClimberCoeff.kI)) { m_leftClimberPIDController.SetI(i); m_leftClimberCoeff.kI = i; }
  if ((d != m_leftClimberCoeff.kD)) { m_leftClimberPIDController.SetD(d); m_leftClimberCoeff.kD = d; }
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
