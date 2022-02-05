#include "Take.h"

#include <iostream>

#include <fmt/core.h>

#include <math.h>

#include <frc/smartdashboard/SmartDashboard.h>

void Take::DeployIntake() {

}

void Take::ReturnIntake() {

}

void Take::EjectBall() {

}

void Take::TakePIDInit() {
  m_rotateIntakePIDController.SetP(m_rotateIntakeCoeff.kP);
  m_rotateIntakePIDController.SetI(m_rotateIntakeCoeff.kI);
  m_rotateIntakePIDController.SetD(m_rotateIntakeCoeff.kD);
  m_rotateIntakePIDController.SetIZone(m_rotateIntakeCoeff.kIz);
  m_rotateIntakePIDController.SetFF(m_rotateIntakeCoeff.kFF);
  m_rotateIntakePIDController.SetOutputRange(m_rotateIntakeCoeff.kMinOutput, m_rotateIntakeCoeff.kMaxOutput);
}

void Take::TakeDashInit() {
//Rotate intake
  frc::SmartDashboard::PutNumber("Rotate Intake P Gain", m_rotateIntakeCoeff.kP);
  frc::SmartDashboard::PutNumber("Rotate Intake I Gain", m_rotateIntakeCoeff.kI);
  frc::SmartDashboard::PutNumber("Rotate Intake D Gain", m_rotateIntakeCoeff.kD);
  frc::SmartDashboard::PutNumber("Rotate Intake Max Output", m_rotateIntakeCoeff.kMaxOutput);
  frc::SmartDashboard::PutNumber("Rotate Intake Min Output", m_rotateIntakeCoeff.kMinOutput);
}

void Take::TakeDashRead() {
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

  frc::Color dashDetectedColor = m_colorSensor.GetColor();
  double dashIR = m_colorSensor.GetIR();

  frc::SmartDashboard::PutNumber("Red", dashDetectedColor.red);
  frc::SmartDashboard::PutNumber("Green", dashDetectedColor.green);
  frc::SmartDashboard::PutNumber("Blue", dashDetectedColor.blue);
  frc::SmartDashboard::PutNumber("IR", dashIR);
}

char Take::BallColor(){
  frc::Color detectedColor = m_colorSensor.GetColor();
  if (detectedColor.red < detectedColor.blue) {return 'b';}
  if (detectedColor.red > detectedColor.blue) {return 'r';}
  else {return 'E';} //E for error
}
