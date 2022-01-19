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

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
