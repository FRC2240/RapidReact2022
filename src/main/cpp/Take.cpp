#include "Take.h"

#include <iostream>

#include <fmt/core.h>

#include <math.h>

#include <frc/smartdashboard/SmartDashboard.h>

void Take::UptakeStart(double speed) {
m_spinIntakeMotor.Set(speed);
m_uptakeMotor.Set(speed);
}

void Take::UptakeStop() {
  m_spinIntakeMotor.Set(0);
  m_uptakeMotor.Set(0);
}

void Take::DeployIntake() {
  m_rotateIntakePIDController.SetReference(10.17, rev::CANSparkMax::ControlType::kSmartMotion);
}

void Take::ReturnIntake() {
  m_rotateIntakePIDController.SetReference(0.0, rev::CANSparkMax::ControlType::kSmartMotion);
}

// enumified
bool Take::RightColorBall() {
  if (Take::BallColorUptake() == 'b' && Take::TeamColor() == blueTeam) {return true;}
  if (Take::BallColorUptake() == 'r' && Take::TeamColor() == redTeam) {return true;}
  else {return false;}
}

// enumed
int Take::TeamColor() {
  if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed) {return redTeam;}
  if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue) {return blueTeam;}
}


// Formerly an int, now an enum
int Take::ManipulateBall(){

  //Here is a logn block comment on the flow of the system
  //
  //If it's the right color and the waiting room is empty, it holds in waiting room (scenario 1)
  //If it's the right color and the waiting room is full, it holds in the (int|up)take (secnario 2)
  // ---
  //If it's the wrong color and the waiting room is empty it sh(oo|i)ts the ball
  //If it's the wrong color and the waitng room is full, it reverses the (int|up)take

// FIXME
  frc::Color uptakeColor; // = m_uptakeSensor.GetColor();
  frc::Color roomColor; // = m_waitingRoomSensor.GetColor();


  if (Take::RightColorBall() && Take::RoomLiveStatus() == nullBall) {
    //scenario 1
    m_spinIntakeMotor.Set(0.1);
    m_uptakeMotor.Set(0.1);
    return 1;
  }

  if (Take::RightColorBall() && Take::RoomLiveStatus() != nullBall) {
    //secnario 2
    m_spinIntakeMotor.Set(0.1);
    return 2;
  }



  if (!Take::RightColorBall() && Take::RoomLiveStatus() == nullBall){

    //secnario 3
    m_spinIntakeMotor.Set(0.1);
    m_uptakeMotor.Set(0.1);
    m_waitingRoomMotor.Set(0.1);
    return 3;

  }

  if (!Take::RightColorBall() && Take::RoomLiveStatus() != nullBall){
    //secnario 4
    m_spinIntakeMotor.Set(-0.1);
    m_uptakeMotor.Set(-0.1);
    return 4;
  }

}
// Eric's code, probably better but I don't know how it works so...
/*

void Take::UptakeBall() {
  if (uptakeMatchedColor == desiredColor ) {
    if (waitingRoomMatchedColor == desiredColor) {
      //waiting room occupied, uptake should spin for a little bit before stopping
      m_uptakePIDController.SetReference(m_stopOne, rev::CANSparkMax::ControlType::kPosition);
    }

    else if (waitingRoomMatchedColor == nothingDetected) {
      //waiting room unoccupied, spin ball all the way into it
      m_uptakePIDController.SetReference(m_stopTwo, rev::CANSparkMax::ControlType::kPosition);
    }
  }
  else {
    EjectBall(); 
  }
  
}

void Take::EjectBall() {
  if (uptakeMatchedColor == undesiredColor) {
    if (waitingRoomMatchedColor == nothingDetected) {
      //slight issue, we want to shoot the ball here, but shooter initialization is in a different class 
    }

  }

}

void Take::ColorsInit() {
 m_colorMatcher.AddColorMatch(kBlue);
 m_colorMatcher.AddColorMatch(kRed);
}

void Take::SetColor() {
  //figure out how to switch desired color/undesired color from red to green and vice versa
}
*/

//TODO: make it so it doesn't update vars if a ball isn't there
//Theese should be for the last ball to pass through the area

int Take::BallColorUptake(){
  if (Take::UptakeLiveStatus() != nullBall){

  // FIXME
  frc::Color detectedColor; // = m_uptakeSensor.GetColor();
  if (detectedColor.blue > detectedColor.red) {
    if (detectedColor.blue > m_blueFloor) {
      return blueBall;
    }
    else {
      if (detectedColor.red > m_redFloor){
        //How did we get here?
        return redBall;
      }
    }
  }

  if (detectedColor.red > detectedColor.blue) {
    if (detectedColor.red > m_redFloor) {
      return redBall;
    }
    else {
      if (detectedColor.blue > m_blueFloor){
        //How did we get here
        return blueBall;
      }
    }
  }
 }
}


//enumed
char Take::BallColorRoom(){

  if (Take::RoomLiveStatus() != nullBall){
  frc::Color detectedColor = m_waitingRoomSensor.GetColor();

  if (detectedColor.blue > detectedColor.red) {
    if (detectedColor.blue > m_blueFloor) {
      return blueBall;
    }
    else {
      if (detectedColor.red > m_redFloor){
        //How did we get here?
        return redBall;
      }
    }
  }

  if (detectedColor.red > detectedColor.blue) {
    if (detectedColor.red > m_redFloor) {
      return redBall;
    }
    else {
      if (detectedColor.blue > m_blueFloor){
        //How did we get here
        return blueBall;
      }
    }
  }
 }
}

//Theese are for the current status of the sensors

//r for red, b for blue, e for empty and E for error

//enumd
char Take::RoomLiveStatus(){
  // FIXME
  frc::Color detectedColor; // = m_waitingRoomSensor.GetColor();
  if (detectedColor.blue > detectedColor.red) {
    if (detectedColor.blue > m_blueFloor) {
      return blueBall;
    }
    else {
      if (detectedColor.red > m_redFloor){
        //How did we get here?
        return redBall;
      }
    }
  }

  if (detectedColor.red > detectedColor.blue) {
    if (detectedColor.red > m_redFloor) {
      return redBall;
    }
    else {
      if (detectedColor.blue > m_blueFloor){
        //How did we get here
        return blueBall;
      }
    }
  }
  else {
    return nullBall;
  }
}

char Take::UptakeLiveStatus(){
  // FIXME
  frc::Color detectedColor; // = m_uptakeSensor.GetColor();
  if (detectedColor.blue > detectedColor.red) {
    if (detectedColor.blue > m_blueFloor) {
      return blueBall;
    }
    else {
      if (detectedColor.red > m_redFloor){
        //How did we get here?
        return redBall;
      }
    }
  }

  if (detectedColor.red > detectedColor.blue) {
    if (detectedColor.red > m_redFloor) {
      return redBall;
    }
    else {
      if (detectedColor.blue > m_blueFloor){
        //How did we get here
        return blueBall;
      }
    }
  }
}


void Take::TakePIDInit() {
  m_rotateIntakePIDController.SetP(m_rotateIntakeCoeff.kP);
  m_rotateIntakePIDController.SetI(m_rotateIntakeCoeff.kI);
  m_rotateIntakePIDController.SetD(m_rotateIntakeCoeff.kD);
  m_rotateIntakePIDController.SetIZone(m_rotateIntakeCoeff.kIz);
  m_rotateIntakePIDController.SetFF(m_rotateIntakeCoeff.kFF);
  m_rotateIntakePIDController.SetOutputRange(m_rotateIntakeCoeff.kMinOutput, m_rotateIntakeCoeff.kMaxOutput);

  m_rotateIntakePIDController.SetSmartMotionMaxVelocity(kMaxVel);
  m_rotateIntakePIDController.SetSmartMotionMinOutputVelocity(kMinVel);
  m_rotateIntakePIDController.SetSmartMotionMaxAccel(kMaxAcc);
  m_rotateIntakePIDController.SetSmartMotionAllowedClosedLoopError(kAllErr);

  m_uptakePIDController.SetP(m_uptakeCoeff.kP);
  m_uptakePIDController.SetI(m_uptakeCoeff.kI);
  m_uptakePIDController.SetD(m_uptakeCoeff.kD);
  m_uptakePIDController.SetIZone(m_uptakeCoeff.kIz);
  m_uptakePIDController.SetFF(m_uptakeCoeff.kFF);
  m_uptakePIDController.SetOutputRange(m_uptakeCoeff.kMinOutput, m_uptakeCoeff.kMaxOutput);

  m_waitingRoomPIDController.SetP(m_waitingRoomCoeff.kP);
  m_waitingRoomPIDController.SetI(m_waitingRoomCoeff.kI);
  m_waitingRoomPIDController.SetD(m_waitingRoomCoeff.kD);
  m_waitingRoomPIDController.SetIZone(m_waitingRoomCoeff.kIz);
  m_waitingRoomPIDController.SetFF(m_waitingRoomCoeff.kFF);
  m_waitingRoomPIDController.SetOutputRange(m_waitingRoomCoeff.kMinOutput, m_waitingRoomCoeff.kMaxOutput);
}

void Take::TakeDashInit() {
//Rotate intake
  frc::SmartDashboard::PutNumber("Rotate Intake P Gain", m_rotateIntakeCoeff.kP);
  frc::SmartDashboard::PutNumber("Rotate Intake I Gain", m_rotateIntakeCoeff.kI);
  frc::SmartDashboard::PutNumber("Rotate Intake D Gain", m_rotateIntakeCoeff.kD);
  frc::SmartDashboard::PutNumber("Rotate Intake Max Output", m_rotateIntakeCoeff.kMaxOutput);
  frc::SmartDashboard::PutNumber("Rotate Intake Min Output", m_rotateIntakeCoeff.kMinOutput);

  frc::SmartDashboard::PutNumber("Uptake P Gain", m_uptakeCoeff.kP);
  frc::SmartDashboard::PutNumber("Uptake I Gain", m_uptakeCoeff.kI);
  frc::SmartDashboard::PutNumber("Uptake D Gain", m_uptakeCoeff.kD);
  frc::SmartDashboard::PutNumber("Uptake Max Output", m_uptakeCoeff.kMaxOutput);
  frc::SmartDashboard::PutNumber("Uptake Min Output", m_uptakeCoeff.kMinOutput);

  frc::SmartDashboard::PutNumber("Waiting Room P Gain", m_waitingRoomCoeff.kP);
  frc::SmartDashboard::PutNumber("Waiting Room I Gain", m_waitingRoomCoeff.kI);
  frc::SmartDashboard::PutNumber("Waiting Room D Gain", m_waitingRoomCoeff.kD);
  frc::SmartDashboard::PutNumber("Waiting Room Max Output", m_waitingRoomCoeff.kMaxOutput);
  frc::SmartDashboard::PutNumber("Waiting Room Min Output", m_waitingRoomCoeff.kMinOutput);
  
}

void Take::TakeDashRead() {
    double p, i, d, min, max;
     //rotate intake
  // read PID coefficients from SmartDashboard
  p   = frc::SmartDashboard::GetNumber("Rotate Intake P Gain", 0);
  std::cout << "Read Dashboard rotate intake p gain: " << p << "\n";
  i   = frc::SmartDashboard::GetNumber("Rotate Intake I Gain", 0);
  std::cout << "Read Dashboard rotate intake i gain: " << i << "\n";
  d   = frc::SmartDashboard::GetNumber("Rotate Intake D Gain", 0);
  std::cout << "Read Dashboard rotate intake d gain: " << d << "\n";
  min = frc::SmartDashboard::GetNumber("Rotate Intake Min Output", 0);
  max = frc::SmartDashboard::GetNumber("Rotate Intake Max Output", 0);

  // If PID coefficients on SmartDashboard have changed, write new values to controller
  if ((p != m_rotateIntakeCoeff.kP)) { m_rotateIntakePIDController.SetP(p); m_rotateIntakeCoeff.kP = p; }
  if ((i != m_rotateIntakeCoeff.kI)) { m_rotateIntakePIDController.SetI(i); m_rotateIntakeCoeff.kI = i; }
  if ((d != m_rotateIntakeCoeff.kD)) { m_rotateIntakePIDController.SetD(d); m_rotateIntakeCoeff.kD = d; }
  if ((max != m_rotateIntakeCoeff.kMaxOutput) || (min != m_rotateIntakeCoeff.kMinOutput)) { 
    m_rotateIntakePIDController.SetOutputRange(min, max); 
    m_rotateIntakeCoeff.kMinOutput = min; m_rotateIntakeCoeff.kMaxOutput = max; 
  }

  p   = frc::SmartDashboard::GetNumber("Uptake P Gain", 0);
  std::cout << "Read Dashboard uptake p gain: " << p << "\n";
  i   = frc::SmartDashboard::GetNumber("Uptake I Gain", 0);
  std::cout << "Read Dashboard uptake i gain: " << i << "\n";
  d   = frc::SmartDashboard::GetNumber("Uptake D Gain", 0);
  std::cout << "Read Dashboard uptake d gain: " << d << "\n";
  min = frc::SmartDashboard::GetNumber("Uptake Min Output", 0);
  max = frc::SmartDashboard::GetNumber("Uptake Max Output", 0);

  if ((p != m_uptakeCoeff.kP)) { m_uptakePIDController.SetP(p); m_uptakeCoeff.kP = p; }
  if ((i != m_uptakeCoeff.kI)) { m_uptakePIDController.SetI(i); m_uptakeCoeff.kI = i; }
  if ((d != m_uptakeCoeff.kD)) { m_uptakePIDController.SetD(d); m_uptakeCoeff.kD = d; }
  if ((max != m_uptakeCoeff.kMaxOutput) || (min != m_uptakeCoeff.kMinOutput)) { 
    m_uptakePIDController.SetOutputRange(min, max); 
    m_uptakeCoeff.kMinOutput = min; m_uptakeCoeff.kMaxOutput = max; 
  }

  p   = frc::SmartDashboard::GetNumber("Waiting Room P Gain", 0);
  std::cout << "Read Dashboard waiting room p gain: " << p << "\n";
  i   = frc::SmartDashboard::GetNumber("Waiting Room I Gain", 0);
  std::cout << "Read Dashboard waiting room i gain: " << i << "\n";
  d   = frc::SmartDashboard::GetNumber("Waiting Room D Gain", 0);
  std::cout << "Read Dashboard waiting room d gain: " << d << "\n";
  min = frc::SmartDashboard::GetNumber("Waiting Room Min Output", 0);
  max = frc::SmartDashboard::GetNumber("Waiting Room Max Output", 0);

  if ((p != m_waitingRoomCoeff.kP)) { m_waitingRoomPIDController.SetP(p); m_waitingRoomCoeff.kP = p; }
  if ((i != m_waitingRoomCoeff.kI)) { m_waitingRoomPIDController.SetI(i); m_waitingRoomCoeff.kI = i; }
  if ((d != m_waitingRoomCoeff.kD)) { m_waitingRoomPIDController.SetD(d); m_waitingRoomCoeff.kD = d; }
  if ((max != m_waitingRoomCoeff.kMaxOutput) || (min != m_waitingRoomCoeff.kMinOutput)) { 
    m_waitingRoomPIDController.SetOutputRange(min, max); 
    m_waitingRoomCoeff.kMinOutput = min; m_uptakeCoeff.kMaxOutput = max; 
  }
    // FIXME
    frc::Color dashDetectedColorUptake; // = m_uptakeSensor.GetColor();
    double dashUptakeIR; // = m_uptakeSensor.GetIR();

  frc::SmartDashboard::PutNumber("Red", dashDetectedColorUptake.red);
  frc::SmartDashboard::PutNumber("Green", dashDetectedColorUptake.green);
  frc::SmartDashboard::PutNumber("Blue", dashDetectedColorUptake.blue);
  frc::SmartDashboard::PutNumber("IR", dashUptakeIR);

  // FIXME
  frc::Color dashDetectedColorRoom; // = m_waitingRoomSensor.GetColor();
  double dashRoomIR; // = m_waitingRoomSensor.GetIR();

  frc::SmartDashboard::PutNumber("Red", dashDetectedColorRoom.red);
  frc::SmartDashboard::PutNumber("Green", dashDetectedColorRoom.green);
  frc::SmartDashboard::PutNumber("Blue", dashDetectedColorRoom.blue);
  frc::SmartDashboard::PutNumber("IR", dashRoomIR);

}
