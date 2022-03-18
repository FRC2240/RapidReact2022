#include "Climber.h"

// Standard C++ Libraries
#include <iostream>
#include <math.h>

// FIRST Specific Libraries
#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>

// TODO: Add a kill button

// Initialization
Climber::Climber() {
  ClimberPIDInit();
}

// This method moves the arm. All that is really needed ATM.
void Climber::EngageLeft(double throttle) {
  m_leftClimberExtender.Set(throttle);
}


/**
 * Method to initialize the climber dashboard
 */
void Climber::ClimberDashInit(){
  frc::SmartDashboard::PutNumber("Left Climber Extend Min Output", m_leftClimberExtendCoeff.kMinOutput);
  frc::SmartDashboard::PutNumber("Left Climber Extend Max Output", m_leftClimberExtendCoeff.kMaxOutput);
  frc::SmartDashboard::PutNumber("Left Climber Extend Point", m_climbExtendPointL);

  // PID Constants for Right Climber Extension
}

/**
 * Method to read the climber dashboard values
 */
void Climber::ClimberDashRead(){

   m_leftClimberExtendCoeff.kMinOutput = frc::SmartDashboard::GetNumber("Left Climber Extend Min Output", 0.0);
  m_leftClimberExtendCoeff.kMaxOutput = frc::SmartDashboard::GetNumber("Left Climber Extend Max Output", 0.0);
  m_climbExtendPointL = frc::SmartDashboard::GetNumber("Left Climber Extend Point", 0.0);

}
/**
 * Method to initialize PID controllers for the climber
 */

//Not needed due to the use of soft limits.
void Climber::ClimberPIDInit(){}


void Climber::GetEncoderValues() {
  frc::SmartDashboard::PutNumber("Current Left Climber Extension Position: ", m_leftClimberExtender.GetSelectedSensorPosition());
}

void Climber::InitializeEncoders() {
  m_leftClimberExtender.SetSelectedSensorPosition(0.0);
}

void Climber::InitializeSoftLimits() {
  m_leftClimberExtender.SetInverted(true);

  m_leftClimberExtender.ConfigForwardSoftLimitEnable(true);
  m_leftClimberExtender.ConfigReverseSoftLimitEnable(true);
  m_leftClimberExtender.ConfigForwardSoftLimitThreshold(kMaxLeft);
  m_leftClimberExtender.ConfigReverseSoftLimitEnable(kMinLeft);
}

void Climber::TestDashInit() {}

void Climber::TestReadDash() {
  // Smart Motion
  m_rotationL = frc::SmartDashboard::GetNumber("Left Climber Rotation Point", 0.0);
}

