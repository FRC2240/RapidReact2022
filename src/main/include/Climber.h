#pragma once

#include <string>


#include "rev/CANSparkMax.h"
#include "ctre/Phoenix.h"

#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>


#include <wpi/sendable/Sendable.h>
#include <wpi/sendable/SendableHelper.h>

#include <frc/Filesystem.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <wpi/fs.h>


#include <frc/DriverStation.h>
#include <frc/Timer.h>
#include <frc/GenericHID.h>
#include <frc/drive/RobotDriveBase.h>
#include <frc/RobotController.h>
#include <frc/motorcontrol/MotorController.h>
#include <frc/DoubleSolenoid.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include "frc/drive/RobotDriveBase.h"
#include "frc/drive/DifferentialDrive.h"
#include <frc/XboxController.h>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include "frc/smartdashboard/SmartDashboard.h"

#include <frc/controller/PIDController.h>

#include <frc/trajectory/TrajectoryUtil.h>
#include <frc/controller/RamseteController.h>
#include <frc/trajectory/TrajectoryGenerator.h>



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
  void RotateLeft(char dirL); // forwards and backwards. All lowercase
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
};


