#pragma once

#include "rev/CANSparkMax.h"

#include <frc/smartdashboard/SendableChooser.h>
#include "frc/smartdashboard/SmartDashboard.h"



#include "frc/DigitalInput.h" 
//#include <frc/util/color.h>

#include "rev/ColorSensorV3.h"
#include "rev/ColorMatch.h"


class Take {
    public:
  void TakePIDInit();
  void TakeDashRead();
  void TakeDashInit();
  void ColorsInit();
  void SetColor();

  //For testing, not operation
  void UptakeStart(double speed);
  void UptakeStop();

  void DeployIntake();
  void ReturnIntake();
  void EjectBall();
  void UptakeBall();
  char BallColorUptake();
  char BallColorRoom();


    private:
    frc::SendableChooser<std::string> m_chooser;

    //Device IDs
  static const int rotateIntakeMotorDeviceID = 5;
  static const int spinIntakeMotorDeviceID = 6;
  static const int uptakeMotorDeviceID = 9;

  static const int waitingRoomMotorDeviceID = 14;
  


  static constexpr auto i2cPort = frc::I2C::Port::kOnboard;

  //Motors
  rev::CANSparkMax m_rotateIntakeMotor{rotateIntakeMotorDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_spinIntakeMotor{spinIntakeMotorDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_uptakeMotor{uptakeMotorDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_waitingRoomMotor {waitingRoomMotorDeviceID, rev::CANSparkMax::MotorType::kBrushless};

  //Encoders
  rev::SparkMaxRelativeEncoder m_rotateIntakeEncoder = m_rotateIntakeMotor.GetEncoder(); 
  rev::SparkMaxRelativeEncoder m_spinIntakeEncoder = m_spinIntakeMotor.GetEncoder(); 
  rev::SparkMaxRelativeEncoder m_uptakeMotorEncoder = m_uptakeMotor.GetEncoder();
  rev::SparkMaxRelativeEncoder m_waitingRoomMotorEncoder = m_waitingRoomMotor.GetEncoder();

  //PID
  rev::SparkMaxPIDController m_rotateIntakePIDController = m_rotateIntakeMotor.GetPIDController();
  rev::SparkMaxPIDController m_uptakePIDController = m_uptakeMotor.GetPIDController();
  rev::SparkMaxPIDController m_waitingRoomPIDController = m_waitingRoomMotor.GetPIDController(); 

  struct pidCoeff {
    double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
};

pidCoeff m_rotateIntakeCoeff{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; 
pidCoeff m_uptakeCoeff {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
pidCoeff m_waitingRoomCoeff {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; 

  //Sensors
  

  rev::ColorSensorV3 m_uptakeSensor {frc::I2C::Port::kOnboard};
  rev::ColorSensorV3 m_waitingRoomSensor {frc::I2C::Port::kOnboard};

  //Colors
static constexpr frc::Color kBlue = frc::Color(0.143, 0.427, 0.429);
static constexpr frc::Color kRed = frc::Color(0.561, 0.232, 0.114);
static constexpr frc::Color kBlack = frc::Color(0.0, 0.0, 0.0);

frc::Color desiredColor;
frc::Color undesiredColor;
frc::Color nothingDetected = kBlack; 

 frc::Color uptakeDetectedColor = m_uptakeSensor.GetColor();
 frc::Color waitingRoomDetectedColor = m_waitingRoomSensor.GetColor(); 

 rev::ColorMatch m_colorMatcher;

 double confidence = 0.0;
frc::Color uptakeMatchedColor = m_colorMatcher.MatchClosestColor(uptakeDetectedColor, confidence);
frc::Color waitingRoomMatchedColor = m_colorMatcher.MatchClosestColor(waitingRoomDetectedColor, confidence);

double m_stopOne, m_stopTwo;



};
