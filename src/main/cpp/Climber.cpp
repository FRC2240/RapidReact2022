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

  // PID Constants for Left Climber Extension
  frc::SmartDashboard::PutNumber("Left Climber Extend P Gain", m_leftClimberExtendCoeff.kP);
  frc::SmartDashboard::PutNumber("Left Climber Extend I Gain", m_leftClimberExtendCoeff.kI);
  frc::SmartDashboard::PutNumber("Left Climber Extend D Gain", m_leftClimberExtendCoeff.kD);
  frc::SmartDashboard::PutNumber("Left Climber Extend FF Gain", m_leftClimberExtendCoeff.kFF);
  frc::SmartDashboard::PutNumber("Left Climber Extend Min Output", m_leftClimberExtendCoeff.kMinOutput);
  frc::SmartDashboard::PutNumber("Left Climber Extend Max Output", m_leftClimberExtendCoeff.kMaxOutput);
  frc::SmartDashboard::PutNumber("Left Climber Extend Point", m_climbExtendPointL);

  // PID Constants for Right Climber Extension
}

/**
 * Method to read the climber dashboard values
 */
void Climber::ClimberDashRead(){

  // Create variables to hold PID values (p, i, d, min, and max)
  double p, i, d, min, max;
 // Read left climber extend/retract PID values

  m_leftClimberExtendCoeff.kP   = frc::SmartDashboard::GetNumber("Left Climber Extend P Gain", 0.0);
  m_leftClimberExtendCoeff.kI   = frc::SmartDashboard::GetNumber("Left Climber Extend I Gain", 0.0);
  m_leftClimberExtendCoeff.kD   = frc::SmartDashboard::GetNumber("Left Climber Extend D Gain", 0.0);
  m_leftClimberExtendCoeff.kFF  = frc::SmartDashboard::GetNumber("Left Climber Extend FF Gain", 0.0);
  m_leftClimberExtendCoeff.kMinOutput = frc::SmartDashboard::GetNumber("Left Climber Extend Min Output", 0.0);
  m_leftClimberExtendCoeff.kMaxOutput = frc::SmartDashboard::GetNumber("Left Climber Extend Max Output", 0.0);
  m_climbExtendPointL = frc::SmartDashboard::GetNumber("Left Climber Extend Point", 0.0);

}
/**
 * Method to initialize PID controllers for the climber
 */

//To be replaced with constructor
void Climber::ClimberPIDInit(){

  // Set to factory default hardware to prevent unexpected behavior
  m_leftClimberExtender.ConfigFactoryDefault();

  // Configure Sensor Source for Primary PID on both extend/retract controllers
  m_leftClimberExtender.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 10);

  // Set inversion type for both climber extend/retract controllers
  m_leftClimberExtender.SetInverted(TalonFXInvertType::CounterClockwise);

  // Set relevant frame periods to be at least as fast as periodic rate for both extend/retract controllers
  m_leftClimberExtender.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10, 10);
  m_leftClimberExtender.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, 10);

  // Set the peak and nominal outputs for left climber extend/retract controller
  m_leftClimberExtender.ConfigNominalOutputForward(0, 10);
  m_leftClimberExtender.ConfigNominalOutputReverse(0, 10);
  m_leftClimberExtender.ConfigPeakOutputForward(m_leftClimberExtendCoeff.kMaxOutput, 10);
  m_leftClimberExtender.ConfigPeakOutputReverse(m_leftClimberExtendCoeff.kMinOutput, 10);

  // Set Motion Magic gains in slot0 for left climber extend/retract controller
  m_leftClimberExtender.SelectProfileSlot(0, 0);
  m_leftClimberExtender.Config_kF(0, m_leftClimberExtendCoeff.kFF, 10);
  m_leftClimberExtender.Config_kP(0, m_leftClimberExtendCoeff.kP, 10);
  m_leftClimberExtender.Config_kI(0, m_leftClimberExtendCoeff.kI, 10);
  m_leftClimberExtender.Config_kD(0,m_leftClimberExtendCoeff.kD, 10);
  // Set acceleration and vcruise velocity for the left climber extend/retract controller
  m_leftClimberExtender.ConfigMotionCruiseVelocity(1500, 10);
  m_leftClimberExtender.ConfigMotionAcceleration(1500, 10);

  // Set acceleration and vcruise velocity for the left climber extend/retract controller

  // Zero the sensor for both controllers
  m_leftClimberExtender.SetSelectedSensorPosition(0, 0, 10);
}


void Climber::GetEncoderValues() {
  // Puts Encoder Values
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


