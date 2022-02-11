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

#include "frc/drive/DifferentialDrive.h"


#include <frc/DriverStation.h>
#include <frc/XboxController.h>



class Shooter {
 public:
  Shooter(frc::DifferentialDrive* d, frc::XboxController* s, Take* t);
  
  void Fire();
  void InitializePIDControllers();

  void Spit(double vel);

 private:

  double CalculateRPM(double d);
  bool LimelightTracking();
  double LimelightDistance();
  void InitializeDashboard();
  void ReadDashboard();

  frc::DifferentialDrive* m_drive;
  frc::XboxController*    m_stick;
  Take*                   m_take;

  bool  shootMan;
  bool  wrongBall;

  double taLowBound, taHighBound;
  double txLowBound, txHighBound;
  double tyLowBound, tyHighBound;
  double heightOfTarget;
  double heightLimelight;
  double constantLimelightAngle;

  //Limelight init should go here
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
