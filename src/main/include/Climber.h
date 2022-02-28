#pragma once

#include "rev/CANSparkMax.h"
#include "ctre/Phoenix.h"

#include <frc/smartdashboard/SendableChooser.h>
#include "frc/smartdashboard/SmartDashboard.h"

#include <frc/controller/PIDController.h>
#include "frc/Servo.h"

#include <frc/Timer.h>



class Climber {
public:
  Climber();
  //ClimberSubsystem();
  void ClimberPIDInit();
  void ClimberDashRead();
  void ClimberDashInit();
  void ExtendALowerL(double setpointL);
  void ExtendALowerR(double setpointR);
  void RotateLeft(double rotatePointL); // f = forwards, b = backwards. All lowercase
  void RotateRight(double rotatePointR);
  void EngageLeft(double throttle);
  void EngageRight(double throttle);

  void SetLeftServo(double position);
  void SetRightServo(double position);

  void Progress();
  void Kill();
  void Run();

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

  void RotateRThrottle(double throttle);
  void RotateLThrottle(double throttle);
  

private:
  bool CanIProgress(); 

  frc::Servo m_rightExtenderServo {0};
  frc::Servo m_leftExtenderServo {1};

  frc::SendableChooser<std::string> m_chooser;

  frc::Servo m_rightExtenderServo {0};
  frc::Servo m_leftExtenderServo {1};



  // Stolen from Robot.h
  static const int rightClimberRotationNeoDeviceID = 10;
  static const int leftClimberRotationNeoDeviceID = 11;

  rev::CANSparkMax m_rightClimberRotationNeo{rightClimberRotationNeoDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_leftClimberRotationNeo{leftClimberRotationNeoDeviceID, rev::CANSparkMax::MotorType::kBrushless};

  // Climber falcons
  WPI_TalonFX m_rightClimberExtender = {12}; //pretty sure these were flipped
  WPI_TalonFX m_leftClimberExtender = {13};
  TalonFXSensorCollection m_leftClimberExtenderEncoder = m_leftClimberExtender.GetSensorCollection();
  TalonFXSensorCollection m_rightClimberExtenderEncoder = m_rightClimberExtender.GetSensorCollection();

  // Ding dong, you are wrong
  struct pidCoeff {
    double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  };
  
  rev::SparkMaxPIDController m_leftClimberRotatePIDController = m_leftClimberRotationNeo.GetPIDController();
  rev::SparkMaxPIDController m_rightClimberRotatePIDController = m_rightClimberRotationNeo.GetPIDController();

  //thing
  
  rev::SparkMaxRelativeEncoder m_rightClimberEncoder = m_rightClimberRotationNeo.GetEncoder(); 
  rev::SparkMaxRelativeEncoder m_leftClimberEncoder = m_leftClimberRotationNeo.GetEncoder(); 

  //Neos
  pidCoeff m_leftClimberRotateCoeff{0.0003, 0.0, 0.0, 0.0, 0.0, 1.0, -1.0}; // negative to go to center
  pidCoeff m_rightClimberRotateCoeff{0.0003, 0.0, 0.0, 0.0, 0.0, 1.0, -1.0};
  // Smart Motion
  double kLMaxVel = 6000, kLMinVel = 0, kLMaxAcc = 4000, kLAllErr = 2; 
  double kRMaxVel = 6000, kRMinVel = 0, kRMaxAcc = 4000, kRAllErr = 2; 

  //splish splash, your opinion is trash

  //Falcons 
  pidCoeff m_leftClimberExtendCoeff{0.1, 0.0, 0.0, 0.0, 0.0, 1.0, -1.0}; 
  pidCoeff m_rightClimberExtendCoeff{0.1, 0.0, 0.0, 0.0, 0.0, 1.0, -1.0};

  //vars

  double m_rotateSetpointR, m_rotateSetpointL = 0.0;
  //double m_backSetPointR, m_backSetPointL = 0.0;


  double m_climbExtendPointR, m_climbExtendPointL = 0.0;
  //double m_climbLowerPointR, m_climbLowerPointL = 0.0;

  int m_phase = 0;

  //rotation positions
  double centerL = -29.0, centerR = -36.0, highL = -51.0, highR = -22.0; 

  //extension soft limits
  double kMaxLeft = 200000.0, kMinLeft = 2000.0, kMaxRight = 410000.0, kMinRight = 3000.0; 

  //engaged servo positions
  double leftDisengaged = 0.7, rightDisengaged = 0.6; 


  //testing
  double m_rotationR, m_rotationL;

  frc::Timer m_climbTimer; 
  
};


