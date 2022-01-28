#pragma once

#include "rev/CANSparkMax.h"
#include "ctre/Phoenix.h"

#include <frc/smartdashboard/SendableChooser.h>
#include "frc/smartdashboard/SmartDashboard.h"

#include <frc/controller/PIDController.h>


class Climber {
public:
  //ClimberSubsystem();
  void RaiseLeft();
  void ClimberPIDInit();
  void ClimberDashRead();
  void ClimberDashInit();
  void RaiseRight();
  void LowerLeft();
  void LowerRight();
  void RotateLeft(char dirL); // f = forwards, b = backwards. All lowercase
  void RotateRight(char dirR);


private:
  frc::SendableChooser<std::string> m_chooser;


  // Stolen from Robot.h
  static const int rightClimberRotationNeoDeviceID = 10;
  static const int leftClimberRotationNeoDeviceID = 11;

  rev::CANSparkMax m_rightClimberRotationNeo{rightClimberRotationNeoDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_leftClimberRotationNeo{leftClimberRotationNeoDeviceID, rev::CANSparkMax::MotorType::kBrushless};

  // Climber falcons
  WPI_TalonFX m_leftClimberExtender = {12};
  WPI_TalonFX m_rightClimberExtender = {13};

  // Ding dong, you are wrong
  struct pidCoeff {
    double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  };

  //PID pain
  frc2::PIDController m_leftClimberExtendPIDController{0.0, 0.0, 0.0}; //kP, kI, kD
  frc2::PIDController m_rightClimberExtendPIDController{0.0, 0.0, 0.0};
  rev::SparkMaxPIDController m_leftClimberRotatePIDController = m_leftClimberRotationNeo.GetPIDController();
  rev::SparkMaxPIDController m_rightClimberRotatePIDController = m_rightClimberRotationNeo.GetPIDController();

  //thing
  
  rev::SparkMaxRelativeEncoder m_rightClimberEncoder = m_rightClimberRotationNeo.GetEncoder(); 
  rev::SparkMaxRelativeEncoder m_leftClimberEncoder = m_leftClimberRotationNeo.GetEncoder(); 

  //Neos
  pidCoeff m_leftClimberRotateCoeff{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  pidCoeff m_rightClimberRotateCoeff{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  


  //splish splash, your opinion is trash

  //Falcons 
  pidCoeff m_leftClimberExtendCoeff{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; 
  pidCoeff m_rightClimberExtendCoeff{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  //vars

  double forthSetPointR, forthSetPointL;
  double backSetPointR, backSetPointL;
};


