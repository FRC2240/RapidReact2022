#include "Climber.h"

#include <iostream>

#include <fmt/core.h>

#include <math.h>

#include <frc/smartdashboard/SmartDashboard.h>

void Climber::RaiseLeft(){
  m_leftClimberExtender.Set(m_leftClimberExtendPIDController.Calculate(m_leftClimberExtenderEncoder.GetIntegratedSensorPosition(), climbExtendPointL)); //not sure whether to do position or absolute position
  //  m_leftClimberExtendPIDController.SetSetpoint(climbExtendPointL);
}

void Climber::RaiseRight(){
   m_rightClimberExtender.Set(m_rightClimberExtendPIDController.Calculate(m_rightClimberExtenderEncoder.GetIntegratedSensorPosition(), climbExtendPointR));
  //  m_rightClimberExtendPIDController.SetSetpoint(climbExtendPointR);
}

void Climber::LowerLeft(){
   m_leftClimberExtender.Set(m_leftClimberExtendPIDController.Calculate(m_leftClimberExtenderEncoder.GetIntegratedSensorPosition(), climbLowerPointL));

  // m_leftClimberExtendPIDController.SetSetpoint(climbLowerPointL);
}

void Climber::LowerRight(){
   m_rightClimberExtender.Set(m_rightClimberExtendPIDController.Calculate(m_rightClimberExtenderEncoder.GetIntegratedSensorPosition(), climbLowerPointR));

  //  m_rightClimberExtendPIDController.SetSetpoint(climbLowerPointR);
}


void Climber::RotateLeft(char dirL){
  if (dirL == 'f'){ //forwards
    m_leftClimberRotatePIDController.SetReference(forthSetPointL, rev::ControlType::kSmartMotion);
  }
  if (dirL == 'b'){ //backwards
    m_leftClimberRotatePIDController.SetReference(backSetPointL, rev::ControlType::kSmartMotion);
  }
}
void Climber::RotateRight(char dirR){
  if (dirR == 'f'){ //forwards
    m_rightClimberRotatePIDController.SetReference(forthSetPointR, rev::ControlType::kSmartMotion);
  }
  if (dirR == 'b'){ //backwards
    m_rightClimberRotatePIDController.SetReference(backSetPointR, rev::ControlType::kSmartMotion);
  }
}


void Climber::ClimberDashInit(){
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
}

void Climber::ClimberDashRead(){
  double p, i, d, min, max;

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


  //falcons (climber extension)
  m_rightClimberExtendPIDController.SetP(m_rightClimberExtendCoeff.kP);
  m_rightClimberExtendPIDController.SetI(m_rightClimberExtendCoeff.kI);
  m_rightClimberExtendPIDController.SetD(m_rightClimberExtendCoeff.kD);

  m_leftClimberExtendPIDController.SetP(m_leftClimberExtendCoeff.kP);
  m_leftClimberExtendPIDController.SetI(m_leftClimberExtendCoeff.kI);
  m_leftClimberExtendPIDController.SetD(m_leftClimberExtendCoeff.kD);


}
