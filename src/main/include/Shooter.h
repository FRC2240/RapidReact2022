#pragma once

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include "frc/smartdashboard/SmartDashboard.h"
#include "frc/drive/DifferentialDrive.h"
#include <frc/XboxController.h> 
#include "ctre/Phoenix.h"
#include "Take.h"

class Shooter {
 public:
  Shooter(frc::DifferentialDrive* d, frc::XboxController* s, Take* t);
  
  void Fire(double m);
  void Reset();
  void InitializePIDControllers();
  void InitializeDashboard();
  void ReadDashboard();
  void ManualShoot(); 
  void Dump();
  void Go();

 private:

  double CalculateRPM(double ty);
  bool LimelightTracking();

  frc::DifferentialDrive* m_drive;
  frc::XboxController*    m_stick;
  Take*                   m_take;

  double m_overrideRPM;
  double kHeightOfTarget   = 103.0;
  double kHeightLimelight  = 28.0;
  double kLimelightAngle   = 13.861;
  double kRadiusOfTarget   = 26.7;
  int m_phaseDelay = 0; 
  double m_scalar = 10.0; 


  WPI_TalonFX m_shootingMotorAlpha {21};
  WPI_TalonFX m_shootingMotorBeta {20};

  TalonFXSensorCollection m_shootingMotorAlphaEncoder = m_shootingMotorAlpha.GetSensorCollection();
  TalonFXSensorCollection m_shootingMotorBetaEncoder = m_shootingMotorBeta.GetSensorCollection();

  struct pidCoeff {
    double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  };

  pidCoeff m_shooterCoeff{0.5, 0.000001, 0.0, 0.0, 0.075, 1.0, -1.0};

  std::shared_ptr<nt::NetworkTable> m_table = nt::NetworkTableInstance::GetDefault().GetTable("limelight-brute");


};
