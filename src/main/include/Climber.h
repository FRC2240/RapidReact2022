#pragma once

#include "rev/CANSparkMax.h"
#include "ctre/Phoenix.h"
#include <string>

#include "frc2/command/SubsystemBase.h" 

class Climber {
public:
  //ClimberSubsystem();
  void RaiseLeft();
  //  void ClimberPIDInit();
  //  void ClimberDashRead();
  void RaiseRight();
  void LowerLeft();
  void LowerRight();
  void RotateLeft(char dirL); // forwards and backwards. All lowercase
  void RotateRight(char dirR);
};
