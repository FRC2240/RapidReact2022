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
  bool intakeBool = 0;
    // 0 = not running, 1 = running

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

 if (m_stick.GetAButtonPressed() == 1) {
   if (intakeBool == 1) {
     // Is running, turn it off

     m_spinIntakeMotor.Set(0);
     m_uptakeMotor.Set(0);

     //TODO retract intake via PIDs here

     intakeBool = 0;
}
   if (intakeBool == 0) {
     // Not running, turn it on

     //TODO deploy intake

     m_spinIntakeMotor.Set(-0.5);
     m_uptakeMotor.Set(0.5);

     intakeBool = 1;
   }
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
