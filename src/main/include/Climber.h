#pragma once

#include "rev/CANSparkMax.h"
#include "ctre/Phoenix.h"

#include "frc2/command/SubsystemBase.h" 

class ClimberSubsystem : public frc2::SubsystemBase {
public: 
ClimberSubsystem();

void raiseLeft();

};