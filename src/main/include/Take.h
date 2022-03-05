#pragma once

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>
#include "rev/CANSparkMax.h"
#include "rev/ColorSensorV3.h"

#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardWidget.h>
#include <frc/shuffleboard/ShuffleboardValue.h>
#include <frc/shuffleboard/ShuffleboardTab.h>

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

class Take
{
public:

  Take(); 
  
  void TakePIDInit();
  void TakeDashRead();
  void TakeDashInit();
  void ColorsInit();
  void SetColor();
  void Feed(double speed);



  void InitializeEncoders();
  void ReadEncoders();

  void Run(bool toggle, bool shooting, frc::DriverStation::Alliance alliance);


  // For testing, not operation

  void UptakeStart(double speed);
  void UptakeStop();

  void DeployIntake();
  void ReturnIntake();

  void TestDashInit();
  void TestDashRead();
  void SetIntakePosition(double position);
  void TestRotation();

  void AutoRunIntake(double speed);
  void AutoStopIntake();


  enum BallColor
  {
    blueBall,
    redBall,
    nullBall,
  };

  enum IntakeState {
    Off,
    Intaking,
    Ejecting,
  };

private:

  // Determine the ball color from Color Sensor value
  BallColor Color(frc::Color);

  // Compare ball color with alliance color
  bool WrongColor(BallColor ball, frc::DriverStation::Alliance alliance);

  // Read Color Sensors
  void ReadSensors();

  // Device IDs
  static const int rotateIntakeMotorDeviceID = 5;
  static const int spinIntakeMotorDeviceID   = 6;
  static const int uptakeMotorDeviceID       = 9;
  static const int waitingRoomMotorDeviceID  = 14;

  bool intakeRunning = false;

  // Shuffleboard  Shuffleboard::Shuffleboard m_shuffleboard; // Shuffleboard
  frc::DriverStation::Alliance m_alliance;

  /* Shuffleboard */
  nt::NetworkTableEntry m_roomBallRedBoard = frc::Shuffleboard::GetTab("Drive Core")

    .Add("Room Ball Red", false)

    .WithWidget("Toggle Button")

    .GetEntry();

  nt::NetworkTableEntry m_roomBallBlueBoard = frc::Shuffleboard::GetTab("Drive Core")

    .Add("Room Ball Blue", false)

    .WithWidget("Boolean Box")

    .GetEntry();
  nt::NetworkTableEntry m_uptakeBallRedBoard = frc::Shuffleboard::GetTab("Drive Core")

    .Add("Uptake Ball Red", false)

    .WithWidget("Boolean Box")

    .GetEntry();
  nt::NetworkTableEntry m_uptakeBallBlueBoard = frc::Shuffleboard::GetTab("Drive Core")

    .Add("Uptake Ball Blue", false)

    .WithWidget("Boolean Box")

    .GetEntry();

  // Motors
  rev::CANSparkMax m_rotateIntakeMotor{rotateIntakeMotorDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_spinIntakeMotor{spinIntakeMotorDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_uptakeMotor{uptakeMotorDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_waitingRoomMotor{waitingRoomMotorDeviceID, rev::CANSparkMax::MotorType::kBrushless};

  // Encoders
  rev::SparkMaxRelativeEncoder m_rotateIntakeEncoder     = m_rotateIntakeMotor.GetEncoder();
  rev::SparkMaxRelativeEncoder m_spinIntakeEncoder       = m_spinIntakeMotor.GetEncoder();
  rev::SparkMaxRelativeEncoder m_uptakeMotorEncoder      = m_uptakeMotor.GetEncoder();
  rev::SparkMaxRelativeEncoder m_waitingRoomMotorEncoder = m_waitingRoomMotor.GetEncoder();

  // PID
  rev::SparkMaxPIDController m_rotateIntakePIDController = m_rotateIntakeMotor.GetPIDController();
  rev::SparkMaxPIDController m_uptakePIDController       = m_uptakeMotor.GetPIDController();
  rev::SparkMaxPIDController m_waitingRoomPIDController  = m_waitingRoomMotor.GetPIDController();

  struct pidCoeff
  {
    double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  };

  double kMaxVel = 4000, kMinVel = 0, kMaxAcc = 2500, kAllErr = 0.2;
  pidCoeff m_rotateIntakeCoeff{3.0e-4, 0.0, 0.0, 0.0, 0.0, 1.0, -1.0};

  pidCoeff m_uptakeCoeff{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  pidCoeff m_waitingRoomCoeff{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  // Sensors
  rev::ColorSensorV3 m_uptakeSensor {frc::I2C::Port::kOnboard};
  rev::ColorSensorV3 m_waitingRoomSensor {frc::I2C::Port::kMXP};

  BallColor m_waitingRoomState = nullBall;
  BallColor m_uptakeState      = nullBall;

  IntakeState m_state = Off;

  int m_ejectTimer = 0;

  // testing rotation PIDs
  double m_rotationPosition; 
};
