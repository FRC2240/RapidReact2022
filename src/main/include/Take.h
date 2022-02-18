#pragma once

#include "rev/CANSparkMax.h"

#include <frc/smartdashboard/SendableChooser.h>
#include "frc/smartdashboard/SmartDashboard.h"
#include <frc/DriverStation.h>

#include "frc/DigitalInput.h"
//#include <frc/util/color.h>

#include "rev/ColorSensorV3.h"
#include "rev/ColorMatch.h"

class Take
{
public:
  explicit Take();
  void TakePIDInit();
  void TakeDashRead();
  void TakeDashInit();
  void ColorsInit();
  void SetColor();

  void Run(bool toggle);

  // For testing, not operation
  void UptakeStart(double speed);
  void UptakeStop();

  void DeployIntake();
  void ReturnIntake();

  int ManipulateBall();

  void UptakeBall();

  // Probably should be private
  int TeamColor();

  bool RightColorBall();

  enum teamColorEnum
  {
    redTeam,
    blueTeam
  };
  enum BallStatusEnum
  {
    rightEmpty,
    rightFull,
    wrongEmpty,
    wrongFull
  };
  enum BallColor
  {
    blueBall,
    redBall,
    nullBall,
    errorBall
  };
  enum TakeExitMessages
  {
    exitNominal,
    exitError
  };

  BallColor BallColorUptake();
  BallColor BallColorRoom();

  // What these mean:
  //  exitNominal: a filler exit message for when things go well
  //  exitNull: an exit message for when nothing happens
  //  exitException: an exit message for when something goes wrong
  //  exitUnexpected: for when something happens that shouldn't happen
private:

  // (right|wrong) means ball color
  // (Empty | Full) means intake status

  // Device IDs
  static const int rotateIntakeMotorDeviceID = 5;
  static const int spinIntakeMotorDeviceID   = 6;
  static const int uptakeMotorDeviceID       = 9;
  static const int waitingRoomMotorDeviceID  = 14;

  //static constexpr auto i2cPort = frc::I2C::Port::kOnboard;

  bool intakeRunning = false;

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

  double kMaxVel = 4000, kMinVel = 0, kMaxAcc = 2500, kAllErr = 0;
  pidCoeff m_rotateIntakeCoeff{3.0e-4, 0.0, 0.0, 0.0, 0.0, 1.0, -1.0};

  pidCoeff m_uptakeCoeff{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  pidCoeff m_waitingRoomCoeff{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  // Sensors
  rev::ColorSensorV3 m_uptakeSensor {frc::I2C::Port::kOnboard};
  //rev::ColorSensorV3 m_waitingRoomSensor {frc::I2C::Port::kMXP};

  // Colors
  static constexpr frc::Color kBlue = frc::Color(0.143, 0.427, 0.429);
  static constexpr frc::Color kRed = frc::Color(0.561, 0.232, 0.114);
  static constexpr frc::Color kBlack = frc::Color(0.0, 0.0, 0.0);


  frc::Color desiredColor;
  frc::Color undesiredColor;
  frc::Color nothingDetected = kBlack;

  rev::ColorMatch m_colorMatcher;

  double m_colorConfidence = 0.0;

  double m_stopOne, m_stopTwo;
};
