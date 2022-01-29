#pragma once

#include "rev/CANSparkMax.h"

#include <frc/smartdashboard/SendableChooser.h>
#include "frc/smartdashboard/SmartDashboard.h"

#include "frc/DigitalInput.h" 
#include "rev/ColorSensorV3.h"


class Take {
    public:
    void TakePIDInit();
    void TakeDashRead();
    void TakeDashInit();

    void DeployIntake();
    void ReturnIntake();
    void EjectBall();

    private:
    frc::SendableChooser<std::string> m_chooser;

    //Device IDs
  static const int rotateIntakeMotorDeviceID = 5;
  static const int spinIntakeMotorDeviceID = 6;
  static const int uptakeMotorDeviceID = 9;
  static const int uptakeIdleMotorDeviceID = 14;
  
  //Motors
  rev::CANSparkMax m_rotateIntakeMotor{rotateIntakeMotorDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_spinIntakeMotor{spinIntakeMotorDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_uptakeMotor{uptakeMotorDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_uptakeIdleMotor {uptakeIdleMotorDeviceID, rev::CANSparkMax::MotorType::kBrushless};

  //Encoders
  rev::SparkMaxRelativeEncoder m_rotateIntakeEncoder = m_rotateIntakeMotor.GetEncoder(); 
  rev::SparkMaxRelativeEncoder m_spinIntakeEncoder = m_spinIntakeMotor.GetEncoder(); 
  rev::SparkMaxRelativeEncoder m_uptakeMotorEncoder = m_uptakeMotor.GetEncoder();
  rev::SparkMaxRelativeEncoder m_uptakeIdleMotorEncoder = m_uptakeIdleMotor.GetEncoder();

  //PID
  rev::SparkMaxPIDController m_rotateIntakePIDController = m_rotateIntakeMotor.GetPIDController();

  struct pidCoeff {
    double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
};

pidCoeff m_rotateIntakeCoeff{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; 

  //Sensors
  frc::DigitalInput m_limitSwitch{0};
  
  rev::ColorSensorV3 m_colorSensor (frc::I2C::Port);

};