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

#include "ctre/Phoenix.h"



class Shooter {
 public:
  Shooter(frc::DifferentialDrive* d, frc::XboxController* s, Take* t);
  
  void Fire();
  void InitializePIDControllers();
  void InitializeDashboard();
  void ReadDashboard();

 private:

  double CalculateRPM(double d);
  bool LimelightTracking();
  double LimelightDistance();
  

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

  WPI_TalonFX m_shootingMotorAlpha {21};
  WPI_TalonFX m_shootingMotorBeta {20};

  TalonFXSensorCollection m_shootingMotorAlphaEncoder = m_shootingMotorAlpha.GetSensorCollection();
  TalonFXSensorCollection m_shootingMotorBetaEncoder = m_shootingMotorBeta.GetSensorCollection();

  struct pidCoeff {
    double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  };

  pidCoeff m_shooterAlphaCoeff{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  pidCoeff m_shooterBetaCoeff{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
};
