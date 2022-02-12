#include "Climber.h"

#include <iostream>

#include <fmt/core.h>

#include <math.h>

#include <frc/smartdashboard/SmartDashboard.h>

//need a kill button 



void Climber::ExtendALowerL(double setpointL) {
  m_climbExtendPointL = setpointL;
  m_leftClimberExtender.Set(ControlMode::MotionMagic, m_climbExtendPointL*2048.0);
}
void Climber::ExtendALowerR(double setpointR) {
  m_climbExtendPointR = setpointR;
  m_rightClimberExtender.Set(ControlMode::MotionMagic, m_climbExtendPointR*2048.0);
}


void Climber::RotateLeft(double rotatePointL){
  m_rotateSetpointL = rotatePointL;
 m_leftClimberRotatePIDController.SetReference(m_rotateSetpointL, rev::CANSparkMax::ControlType::kSmartMotion);
}
void Climber::RotateRight(double rotatePointR){
  m_rotateSetpointR = rotatePointR;
  m_rightClimberRotatePIDController.SetReference(m_rotateSetpointR, rev::CANSparkMax::ControlType::kSmartMotion);
}


void Climber::ClimberDashInit(){
   //falcons
  frc::SmartDashboard::PutNumber("Left Climber Extend P Gain", m_leftClimberExtendCoeff.kP);
  frc::SmartDashboard::PutNumber("Left Climber Extend I Gain", m_leftClimberExtendCoeff.kI);
  frc::SmartDashboard::PutNumber("Left Climber Extend D Gain", m_leftClimberExtendCoeff.kD);
  frc::SmartDashboard::PutNumber("Left Climber Extend FF Gain", m_leftClimberExtendCoeff.kFF);
  frc::SmartDashboard::PutNumber("Left Climber Extend Min Output", m_leftClimberExtendCoeff.kMinOutput);
  frc::SmartDashboard::PutNumber("Left Climber Extend Max Output", m_leftClimberExtendCoeff.kMaxOutput);
  frc::SmartDashboard::PutNumber("Left Climber Extend Point", m_climbExtendPointL);
  //frc::SmartDashboard::PutNumber("Left Climber Lower Point", m_climbLowerPointL);

  frc::SmartDashboard::PutNumber("Right Climber Extend P Gain", m_rightClimberExtendCoeff.kP);
  frc::SmartDashboard::PutNumber("Right Climber Extend I Gain", m_rightClimberExtendCoeff.kI);
  frc::SmartDashboard::PutNumber("Right Climber Extend D Gain", m_rightClimberExtendCoeff.kD);
  frc::SmartDashboard::PutNumber("Right Climber Extend FF Gain", m_rightClimberExtendCoeff.kFF);
  frc::SmartDashboard::PutNumber("Right Climber Extend Min Output", m_rightClimberExtendCoeff.kMinOutput);
  frc::SmartDashboard::PutNumber("Right Climber Extend Max Output", m_rightClimberExtendCoeff.kMaxOutput);
  frc::SmartDashboard::PutNumber("Right Climber Extend Point", m_climbExtendPointR);
  //frc::SmartDashboard::PutNumber("Right Climber Lower Point", m_climbLowerPointR);

  //rotate climber
  frc::SmartDashboard::PutNumber("Left Climber Rotate P Gain", m_leftClimberExtendCoeff.kP);
  frc::SmartDashboard::PutNumber("Left Climber Rotate I Gain", m_leftClimberExtendCoeff.kI);
  frc::SmartDashboard::PutNumber("Left Climber Rotate D Gain", m_leftClimberExtendCoeff.kD);
  frc::SmartDashboard::PutNumber("Left Climber Rotate Max Output", m_leftClimberExtendCoeff.kMaxOutput);
  frc::SmartDashboard::PutNumber("Left Climber Rotate Min Output", m_leftClimberExtendCoeff.kMinOutput);
  frc::SmartDashboard::PutNumber("Left Climber Rotate Point", m_rotateSetpointL);
  //frc::SmartDashboard::PutNumber("Left Climber Rotate Back Point", m_backSetPointL);

  frc::SmartDashboard::PutNumber("Right Climber Rotate P Gain", m_rightClimberExtendCoeff.kP);
  frc::SmartDashboard::PutNumber("Right Climber Rotate I Gain", m_rightClimberExtendCoeff.kI);
  frc::SmartDashboard::PutNumber("Right Climber Rotate D Gain", m_rightClimberExtendCoeff.kD);
  frc::SmartDashboard::PutNumber("Right Climber Rotate Max Output", m_rightClimberExtendCoeff.kMaxOutput);
  frc::SmartDashboard::PutNumber("Right Climber Rotate Min Output", m_rightClimberExtendCoeff.kMinOutput);
  frc::SmartDashboard::PutNumber("Right Climber Rotate Point", m_rotateSetpointR);
  //frc::SmartDashboard::PutNumber("Right Climber Rotate Back Point", m_backSetPointR);
}

void Climber::ClimberDashRead(){
  double p, i, d, min, max;

// Climber Rotation
  p   = frc::SmartDashboard::GetNumber("Right Climber Rotate P Gain", 0.0);
  std::cout << "Read Dashboard Right Rotate Climber p gain: " << p << "\n";
  i   = frc::SmartDashboard::GetNumber("Right Climber Rotate I Gain", 0.0);
  std::cout << "Read Dashboard Right Climber Rotate i gain: " << i << "\n";
  d   = frc::SmartDashboard::GetNumber("Right Climber Rotate D Gain", 0.0);
  std::cout << "Read Dashboard Right Climber Rotate d gain: " << d << "\n";
  min = frc::SmartDashboard::GetNumber("Right Climber Rotate Min Output", 0.0);
  max = frc::SmartDashboard::GetNumber("Right Climber Rotate Max Output", 0.0);
  m_rotateSetpointL = frc::SmartDashboard::GetNumber("Left Climber Rotate Point", 0.0);
  m_rotateSetpointR = frc::SmartDashboard::GetNumber("Right Climber Rotate Point", 0.0);
  //m_backSetPointL = frc::SmartDashboard::GetNumber("Left Climber Rotate Back Point", 0.0);
 // m_backSetPointR = frc::SmartDashboard::GetNumber("Right Climber Rotate Back Point", 0.0);

  if ((p != m_rightClimberRotateCoeff.kP)) { m_rightClimberRotatePIDController.SetP(p);m_rightClimberRotateCoeff.kP = p; }
  if ((i != m_rightClimberRotateCoeff.kI)) { m_rightClimberRotatePIDController.SetI(i); m_rightClimberRotateCoeff.kI = i; }
  if ((d != m_rightClimberRotateCoeff.kD)) { m_rightClimberRotatePIDController.SetD(d); m_rightClimberRotateCoeff.kD = d; }
  if ((max != m_rightClimberRotateCoeff.kMaxOutput) || (min != m_rightClimberRotateCoeff.kMinOutput)) { 
    m_rightClimberRotatePIDController.SetOutputRange(min, max); 
    m_rightClimberRotateCoeff.kMinOutput = min; m_rightClimberRotateCoeff.kMaxOutput = max; 
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
  m_leftClimberExtendCoeff.kP   = frc::SmartDashboard::GetNumber("Left Climber Extend P Gain", 0.0);
  m_leftClimberExtendCoeff.kI   = frc::SmartDashboard::GetNumber("Left Climber Extend I Gain", 0.0);
  m_leftClimberExtendCoeff.kD   = frc::SmartDashboard::GetNumber("Left Climber Extend D Gain", 0.0);
  m_leftClimberExtendCoeff.kFF  = frc::SmartDashboard::GetNumber("Left Climber Extend FF Gain", 0.0);
  m_leftClimberExtendCoeff.kMinOutput = frc::SmartDashboard::GetNumber("Left Climber Extend Min Output", 0.0);
  m_leftClimberExtendCoeff.kMaxOutput = frc::SmartDashboard::GetNumber("Left Climber Extend Max Output", 0.0);
  m_climbExtendPointL = frc::SmartDashboard::GetNumber("Left Climber Extend Point", 0.0);
  //m_climbLowerPointL = frc::SmartDashboard::GetNumber("Left Climber Lower Point", 0.0);

  m_rightClimberExtendCoeff.kP   = frc::SmartDashboard::GetNumber("Right Climber Extend P Gain", 0.0);
  m_rightClimberExtendCoeff.kI   = frc::SmartDashboard::GetNumber("Right Climber Extend I Gain", 0.0);
  m_rightClimberExtendCoeff.kD   = frc::SmartDashboard::GetNumber("Right Climber Extend D Gain", 0.0);
  m_rightClimberExtendCoeff.kFF  = frc::SmartDashboard::GetNumber("Right Climber Extend FF Gain", 0.0);
  m_rightClimberExtendCoeff.kMinOutput = frc::SmartDashboard::GetNumber("Right Climber Extend Min Output", 0.0);
  m_rightClimberExtendCoeff.kMaxOutput = frc::SmartDashboard::GetNumber("Right Climber Extend Max Output", 0.0);
  m_climbExtendPointR = frc::SmartDashboard::GetNumber("Right Climber Extend Point", 0.0);
  //m_climbLowerPointR = frc::SmartDashboard::GetNumber("Right Climber Lower Point", 0.0);

  
}

void Climber::ClimberPIDInit(){
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

  /* Factory default hardware to prevent unexpected behavior */
  m_leftClimberExtender.ConfigFactoryDefault();
  m_rightClimberExtender.ConfigFactoryDefault();

    /* Configure Sensor Source for Pirmary PID */
  m_leftClimberExtender.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 10);
  m_rightClimberExtender.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 10);

    //_talon.SetSensorPhase(false);
    m_leftClimberExtender.SetInverted(TalonFXInvertType::CounterClockwise);
    m_rightClimberExtender.SetInverted(TalonFXInvertType::CounterClockwise);

    /* Set relevant frame periods to be at least as fast as periodic rate */
    m_leftClimberExtender.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10, 10);
    m_leftClimberExtender.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, 10);

    m_rightClimberExtender.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10, 10);
    m_rightClimberExtender.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, 10);

    /* Set the peak and nominal outputs */
    m_leftClimberExtender.ConfigNominalOutputForward(0, 10);
    m_leftClimberExtender.ConfigNominalOutputReverse(0, 10);
    m_leftClimberExtender.ConfigPeakOutputForward(m_leftClimberExtendCoeff.kMaxOutput, 10);
    m_leftClimberExtender.ConfigPeakOutputReverse(m_leftClimberExtendCoeff.kMinOutput, 10);

    m_rightClimberExtender.ConfigNominalOutputForward(0, 10);
    m_rightClimberExtender.ConfigNominalOutputReverse(0, 10);
    m_rightClimberExtender.ConfigPeakOutputForward(m_rightClimberExtendCoeff.kMaxOutput, 10);
    m_rightClimberExtender.ConfigPeakOutputReverse(m_rightClimberExtendCoeff.kMinOutput, 10);

    /* Set Motion Magic gains in slot0 - see documentation */
    m_leftClimberExtender.SelectProfileSlot(0, 0);
    m_leftClimberExtender.Config_kF(0, m_leftClimberExtendCoeff.kFF, 10);
    m_leftClimberExtender.Config_kP(0, m_leftClimberExtendCoeff.kP, 10);
    m_leftClimberExtender.Config_kI(0, m_leftClimberExtendCoeff.kI, 10);
    m_leftClimberExtender.Config_kD(0,m_leftClimberExtendCoeff.kD, 10);

    m_rightClimberExtender.SelectProfileSlot(0, 0);
    m_rightClimberExtender.Config_kF(0, m_rightClimberExtendCoeff.kFF, 10);
    m_rightClimberExtender.Config_kP(0, m_rightClimberExtendCoeff.kP, 10);
    m_rightClimberExtender.Config_kI(0, m_rightClimberExtendCoeff.kI, 10);
    m_rightClimberExtender.Config_kD(0,m_rightClimberExtendCoeff.kD, 10);

    /* Set acceleration and vcruise velocity - see documentation */
    m_leftClimberExtender.ConfigMotionCruiseVelocity(1500, 10);
    m_leftClimberExtender.ConfigMotionAcceleration(1500, 10);

    m_rightClimberExtender.ConfigMotionCruiseVelocity(1500, 10);
    m_rightClimberExtender.ConfigMotionAcceleration(1500, 10);

    /* Zero the sensor */
    m_leftClimberExtender.SetSelectedSensorPosition(0, 0, 10);

    m_rightClimberExtender.SetSelectedSensorPosition(0, 0, 10);
}

void Climber::Progress() {
  bool ok = CanIProgress();
  if (ok) {
    m_phase++;
  }
  else {
    m_phase--; 
    //not sure about this line yet, either this or we create another function that resets the bot to a safe state (e.g. Climber::Reset())
  }
}

bool Climber::CanIProgress() {
  switch(m_phase) {
    case 0:
    break;
    
    case 1:
    return m_leftClimberExtender.GetSelectedSensorPosition() == phaseOneLift; 

    break;

    case 2:
    return m_leftClimberExtender.GetSelectedSensorPosition() == phaseTwoRetract;

    break;

    case 3:
    return m_rightClimberEncoder.GetPosition() == phaseThreeRotate;

    break;

    case 4:
    return m_leftClimberExtender.GetSelectedSensorPosition() == phaseFourRetract;

    break;

    case 5:
    return m_rightClimberExtender.GetSelectedSensorPosition() == phaseFiveExtend;

    break;

    case 6:
    return m_rightClimberExtender.GetSelectedSensorPosition() == phaseSixRetract;

    break;

    case 7:
    return m_rightClimberExtender.GetSelectedSensorPosition() == phaseSevenRetract 
    && m_leftClimberExtender.GetSelectedSensorPosition() == phaseSevenExtend;

    break;

    case 8:
    return m_leftClimberEncoder.GetPosition() == phaseEightRotate;

    break;

    case 9:
    return m_leftClimberExtender.GetSelectedSensorPosition() == phaseNineRetract;

    break;

  }

}

void Climber::Kill() {
  m_phase = 0;
}

void Climber::Run() {
  switch(m_phase) {
    case 0:

    case 1: 
    //raise left arm --> driver then drives robot towards bars
    ExtendALowerL(phaseOneLift);

    break;

    case 2:
    //retract left arm till hooked on bar
    ExtendALowerL(phaseTwoRetract);

    break;

    case 3:
    //rotate right bar
    RotateRight(phaseThreeRotate);

    break;

    case 4:
    //retract left arm further
    ExtendALowerL(phaseFourRetract);

    break;

    case 5:
    //extend right bar
    ExtendALowerR(phaseFiveExtend);

    break;

    case 6: 
    //retract right bar until it's hooked
    ExtendALowerR(phaseSixRetract);

    break;

    case 7: 
    //retract right and extend left until left is unhooked
    ExtendALowerR(phaseSevenRetract);
    ExtendALowerL(phaseSevenExtend);

    break;

    case 8: 
    //rotate left
    RotateLeft(phaseEightRotate);

    break;

    case 9: 
    //retract left until it's hooked
    ExtendALowerL(phaseNineRetract);

    break;

  }

}
