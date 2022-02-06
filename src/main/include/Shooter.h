#pragma once

#include "Take.h"

#include "rev/CANSparkMax.h"

#include <wpi/sendable/Sendable.h>
#include <wpi/sendable/SendableHelper.h>

#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

#include <frc/smartdashboard/SendableChooser.h>
#include "frc/smartdashboard/SmartDashboard.h"

#include <frc/DriverStation.h>
#include <frc/XboxController.h>



class Shooter {
 public:
  bool LimelightTracking();
  double CalculateRPM(double d);
  void ShooterArm();
  void ShooterFire();

  void InitializePIDControllers();
  void InitializeDashboard();
  void ReadDashboard();
 private:

  Take m_take;
  frc::XboxController m_stick{0};

  bool  shootMan;
  bool  wrongBall;

  double taLowBound, taHighBound;
  double txLowBound, txHighBound;
  double tyLowBound, tyHighBound;
  double heightOfTarget;
  double heightLimelight;
  double constantLimelightAngle;

  static const int shootingMotorAlphaDeviceID = 7;
  static const int shootingMotorBetaDeviceID = 8;

  rev::CANSparkMax m_shootingMotorAlpha{shootingMotorAlphaDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_shootingMotorBeta{shootingMotorBetaDeviceID, rev::CANSparkMax::MotorType::kBrushless};

  rev::SparkMaxRelativeEncoder m_shootingMotorAlphaEncoder = m_shootingMotorAlpha.GetEncoder(); 
  rev::SparkMaxRelativeEncoder m_shootingMotorBetaEncoder = m_shootingMotorBeta.GetEncoder();
  double tx_OFFSET = 0.0;

  rev::SparkMaxPIDController m_shooterAlphaPIDController = m_shootingMotorAlpha.GetPIDController();
  rev::SparkMaxPIDController m_shooterBetaPIDController = m_shootingMotorBeta.GetPIDController();

  struct pidCoeff {
    double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  };

  pidCoeff m_shooterAlphaCoeff{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  pidCoeff m_shooterBetaCoeff{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
};
