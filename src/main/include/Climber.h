#pragma once

#include "rev/CANSparkMax.h"
#include "ctre/Phoenix.h"

#include <frc/smartdashboard/SendableChooser.h>
#include "frc/smartdashboard/SmartDashboard.h"

#include <frc/controller/PIDController.h>
#include "frc/Servo.h"
#include <frc/Timer.h>

#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardWidget.h>
#include <frc/shuffleboard/ShuffleboardValue.h>
#include <frc/shuffleboard/ShuffleboardTab.h>

#include <frc/Timer.h>



class Climber {
public:
  Climber();
  void ClimberPIDInit();
  void ClimberDashRead();
  void ClimberDashInit();
  void EngageLeft(double throttle);

  void GetEncoderValues();
  void InitializeEncoders();
  void InitializeSoftLimits();

//Everything for testing
  void TestDashInit();
  void TestReadDash();
  void TestL();
  void TestR();

  void SetPhase(int phase);
  int GetPhase();

private:

  frc::SendableChooser<std::string> m_chooser;

  // Climber falcons
  WPI_TalonFX m_leftClimberExtender = {13};
  TalonFXSensorCollection m_leftClimberExtenderEncoder = m_leftClimberExtender.GetSensorCollection();

  struct pidCoeff {
    double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  };

  // Smart Motion
  double kLMaxVel = 6000, kLMinVel = 0, kLMaxAcc = 4000, kLAllErr = 2; 
  double kRMaxVel = 6000, kRMinVel = 0, kRMaxAcc = 4000, kRAllErr = 2; 


  //Falcons 
  pidCoeff m_leftClimberExtendCoeff{0.1, 0.0, 0.0, 0.0, 0.0, 1.0, -1.0}; 

  double m_rotateSetpointR, m_rotateSetpointL = 0.0;

  double m_climbExtendPointR, m_climbExtendPointL = 0.0;

  int m_phase = 0;
  int phase_delay = 0;
  int phase_delay_redux = 0;

  // NOTE: False means an error cout hasn't happened and true means it has.

  //rotation positions
  double centerL = 35.5, centerR = 20.0, highL = 60.0, highR = 7.5, finalR = 0.0; 

  //extension soft limits
  double kMaxLeft = 254350.0, kMinLeft = 2000.0, kMaxRight = 386000.0, kMinRight = 2000.0; 

  //testing
  double m_rotationR, m_rotationL;

  frc::Timer m_climbTimer;
};

