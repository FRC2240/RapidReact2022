#include "Shooter.h"

// Standard C++ Libraries
#include <iostream>
#include <math.h>

// FIRST Specific Libraries
#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>

/**
 * Constructor for the Shooter Class.
 * 
 * parameters:
 *   d - pointer to a DifferentialDrive object
 *   s - pointer to a XboxController object
 *   t - pointer to a Take object
 */
Shooter::Shooter(frc::DifferentialDrive* d, frc::XboxController* s, Take* t)
    : m_drive(d),
      m_stick(s),
      m_take(t)
{
  // QUESTION: These are being manually called in Robot.cpp, but they are also being called on class instantiation
  // Why is it being repeated? 
  // Initialize Dashboard and PID Controllers
  InitializeDashboard();
  InitializePIDControllers();
}

/**
 *  Method to eject wrong colored balls out of the shooter
 * 
 * parameters:
 *   vel - double representing velocity to spit. Defaults to 0.1
 */
void Shooter::Spit(double vel = 0.1) {
  m_shootingMotorAlpha.Set(vel);
  m_shootingMotorBeta.Set(vel);
}

/**
 * Method for Limelight Tracking. Returns true/false depending on if
 * the target for shooting is in frame
 * 
 * returns:
 *   boolean for target in frame
 */
bool Shooter::LimelightTracking()
{

  // Values for target x, y, and a
  double ta, tx, ty;

  // Convert network table values to doubles, 0.0 if no value
  tx = m_table->GetEntry("tx").GetDouble(0.0);
  ty = m_table->GetEntry("tx").GetDouble(0.0);
  ta = m_table->GetEntry("tx").GetDouble(0.0);

  // Print target x, y, and a to console
  std::cout << tx << ty << ta << "\n";

  // Dummy values
  if (ta <= taHighBound &&
      ta >= taLowBound &&
      tx <= txHighBound &&
      tx >= txLowBound &&
      ty <= tyHighBound &&
      ty >= tyLowBound)
  {
    return true;
  }
  else
  {
    return false;
  }

}

/**
 * Method to calculate shooter RPM. Not defined at the moment
 */
double Shooter::CalculateRPM(double d)
{
  return 0.0; // TODO: Add equation
}

/**
 * Method to fire the shooter
 */
void Shooter::Fire() {

  // If override RPM value in dashboard is set
  if (m_overridenRPM != 0) {
    // Override the shooter RPM
    m_shootingMotorAlpha.Set(ControlMode::Velocity, m_overridenRPM*(2048.0/600.0));
    m_shootingMotorBeta.Set(ControlMode::Velocity, m_overridenRPM*(2048.0/600.0));
  }

  // If Driver Station alliance is RED
  if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed)
  {
    bool wrongBallInSystem = m_take->BallColorRoom() == m_take->redBall;
    if (m_take->BallColorRoom() == m_take->errorBall)
    {
      std::cout << "[WARN]: Color Sensor issue \n";
    }
  }
  // If Driver Station alliance is BLUE
  else if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue)
  {
    bool wrongBallInSystem = m_take->BallColorRoom() == m_take->blueBall;
    if (m_take->BallColorRoom() == m_take->errorBall)
    {
      std::cout << "[WARN]: Color Sensor issue \n";
    }
  }
  // If you reach this state something is wrong
  else
  {
    std::cout << "[ERROR]: Cannot get current alliance from Driver Station";
  }

  // If you have the wrong ball in the system, set the controller rumble.
  if (wrongBallInSystem) {
    m_stick->SetRumble(frc::GenericHID::RumbleType::kLeftRumble, 1.0);
    m_stick->SetRumble(frc::GenericHID::RumbleType::kRightRumble, 1.0);
  }
  // Otherwise set the controller rumble to zero
  else {
    m_stick->SetRumble(frc::GenericHID::RumbleType::kLeftRumble, 0.0);
    m_stick->SetRumble(frc::GenericHID::RumbleType::kRightRumble, 0.0);
  }

  // QUESTION: Why is ta commented out?
  // Doubles for target a, x, y
  double /*ta,*/ tx, ty;

  // Convert network table entries to doubles
  tx = m_table->GetEntry("tx").GetDouble(0.0);
  ty = m_table->GetEntry("ty").GetDouble(0.0);
  // ta = m_table->GetEntry("ta").GetDouble(0.0);

  // Invert the target y
  ty = ty * -1;

  // If manual shooting is enabled
  if (manualShootingEnabled)
  {
    // If limelight tracking can see the target
    if (LimelightTracking() == true)
    {
      //Calculate the distance from the limelight camera to the detected target
      double distance = ((heightOfTarget - heightLimelight) / tan((constantLimelightAngle + ty) * (3.141592653 / 180)));
      
      // NOTE: The method below currently will not work because CalculateRPM only returns 0 currently
      // Calculate the RPM needed for that distance.
      double rpm = CalculateRPM(distance);
      
      // NOTE: The methods below will not currently work because rpm is zero
      // Set the target velocity for both shooting motors
      m_shootingMotorAlpha.Set(ControlMode::Velocity, rpm*(2048.0/600.0));
      m_shootingMotorBeta.Set(ControlMode::Velocity, rpm*(2048.0/600.0));
    }
    // If limelight tracking cannot see the targer
    else {
      // Turn the robot
      if (tx < 0) {
        m_drive->ArcadeDrive(0, 0.5);
      } else if (tx > 0) {
        m_drive->ArcadeDrive(0, -0.5);
      }
    }
  }
  // If manual shooting is NOT enabled
  else {
    //Calculate the distance from the limelight camera to the detected target
    double distance = Shooter::LimelightDistance();

    // NOTE: The method below currently will not work because CalculateRPM only returns 0 currently
    // Calculate the RPM needed for that distance.
    double rpm = CalculateRPM(distance);

    // NOTE: The methods below will not currently work because rpm is zero
    // Set the target velocity for both shooting motors
    m_shootingMotorAlpha.Set(ControlMode::Velocity, rpm*(2048.0/600.0));
    m_shootingMotorBeta.Set(ControlMode::Velocity, rpm*(2048.0/600.0));
  }
}


/**
 * Method to calculate the distance to the target for auto shooting mode
 * returns:
 *   double for the distance from the limelight to the target
 */
double Shooter::LimelightDistance() {

  // Doubles for target a, target x, target y
  double ta, tx, ty;

  // NOTE: Matthew changed these from SetDoubles to GetDoubles
  // Because otherwise you were just always setting tx, ty, and ta
  // to zero based on how this was designed

  // Convert network table entries to double, 
  tx = m_table->GetEntry("tx").GetDouble(0.0);
  ty = m_table->GetEntry("ty").GetDouble(0.0);
  ta = m_table->GetEntry("ta").GetDouble(0.0);

  // Limelight FOV: 49.7
  // Limelight Angle: 36.711
  // Calculate theta = (FOV * ty) + Angle
  double theta = (49.7*ty)+36.711;

  // NOTE: I do not know where the 76.545 comes from

  // Calculate the distance
  double limelightDistance = (76.545/tan(theta));

  return limelightDistance;
}

/**
 * Method to initialize the PID Controllers for the shooter
 */
void Shooter::InitializePIDControllers()
{

  // Read current dashboard values
  ReadDashboard();

  // Factory default hardware to prevent unexpected behavior
  m_shootingMotorAlpha.ConfigFactoryDefault();
  m_shootingMotorBeta.ConfigFactoryDefault();

  // Configure sensor source for Primary PID
  m_shootingMotorAlpha.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 10);
  m_shootingMotorBeta.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 10);

  // Configure motor inversion
  m_shootingMotorAlpha.SetInverted(TalonFXInvertType::CounterClockwise);
  m_shootingMotorBeta.SetInverted(TalonFXInvertType::CounterClockwise);

  // Set relevant frame periods to be at least as fast as periodic rate for motor alpha
  m_shootingMotorAlpha.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10, 10);
  m_shootingMotorAlpha.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, 10);
  
  // Set relevant frame periods to be at least as fast as periodic rate for motor beta
  m_shootingMotorBeta.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10, 10);
  m_shootingMotorBeta.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, 10);

  // Set the peak and nominal outputs for motor alpha
  m_shootingMotorAlpha.ConfigNominalOutputForward(0, 10);
  m_shootingMotorAlpha.ConfigNominalOutputReverse(0, 10);
  m_shootingMotorAlpha.ConfigPeakOutputForward(m_shooterAlphaCoeff.kMaxOutput, 10);
  m_shootingMotorAlpha.ConfigPeakOutputReverse(m_shooterAlphaCoeff.kMinOutput, 10);

  // Set the peak and nominal outputs for motor beta
  m_shootingMotorBeta.ConfigNominalOutputForward(0, 10);
  m_shootingMotorBeta.ConfigNominalOutputReverse(0, 10);
  m_shootingMotorBeta.ConfigPeakOutputForward(m_shooterBetaCoeff.kMaxOutput, 10);
  m_shootingMotorBeta.ConfigPeakOutputReverse(m_shooterBetaCoeff.kMinOutput, 10);

  // Set Motion Magic gains in slot0 for motor alpha - see documentation
  m_shootingMotorAlpha.SelectProfileSlot(0, 0);
  m_shootingMotorAlpha.Config_kF(0, m_shooterAlphaCoeff.kFF, 10);
  m_shootingMotorAlpha.Config_kP(0, m_shooterAlphaCoeff.kP, 10);
  m_shootingMotorAlpha.Config_kI(0, m_shooterAlphaCoeff.kI, 10);
  m_shootingMotorAlpha.Config_kD(0, m_shooterAlphaCoeff.kD, 10);

  // Set Motion Magic gains in slot0 for motor beta - see documentation
  m_shootingMotorBeta.SelectProfileSlot(0, 0);
  m_shootingMotorBeta.Config_kF(0, m_shooterBetaCoeff.kFF, 10);
  m_shootingMotorBeta.Config_kP(0, m_shooterBetaCoeff.kP, 10);
  m_shootingMotorBeta.Config_kI(0, m_shooterBetaCoeff.kI, 10);
  m_shootingMotorBeta.Config_kD(0, m_shooterBetaCoeff.kD, 10);

  // Set acceleration and vcruise velocity for motor alpha - see documentation
  m_shootingMotorAlpha.ConfigMotionCruiseVelocity(1500, 10);
  m_shootingMotorAlpha.ConfigMotionAcceleration(1500, 10);

  // Set acceleration and vcruise velocity for motor beta - see documentation
  m_shootingMotorBeta.ConfigMotionCruiseVelocity(1500, 10);
  m_shootingMotorBeta.ConfigMotionAcceleration(1500, 10);

  // Zero the sensors
  m_shootingMotorBeta.SetSelectedSensorPosition(0, 0, 10);
  m_shootingMotorBeta.SetSelectedSensorPosition(0, 0, 10);
}

/**
 * Initialize the dashboard for the shooter
 */
void Shooter::InitializeDashboard()
{
  // Put the current RPM values for shooter motters
  frc::SmartDashboard::PutNumber("Shooter Alpha RPM", m_shootingMotorAlpha.GetSelectedSensorVelocity());
  frc::SmartDashboard::PutNumber("Shooter Beta RPM", m_shootingMotorBeta.GetSelectedSensorVelocity());

  // If the RPM is overrident, display the overriden RPM
  if (m_overridenRPM != 0) {
    frc::SmartDashboard::PutNumber("Overridden RPM", m_overridenRPM);
  }

  // NOTE: This method will not work because CalculateRPM currently just returns 0
  // Display desired RPM
  frc::SmartDashboard::PutNumber("Desired RPM", CalculateRPM(LimelightDistance()));
  // Display limelight calculated distance to target
  frc::SmartDashboard::PutNumber("Reported Distance", LimelightDistance());

  // Display alpha motor related PID constants
  frc::SmartDashboard::PutNumber("Alpha P Gain", m_shooterAlphaCoeff.kP);
  frc::SmartDashboard::PutNumber("Alpha I Gain", m_shooterAlphaCoeff.kI);
  frc::SmartDashboard::PutNumber("Alpha D Gain", m_shooterAlphaCoeff.kD);
  frc::SmartDashboard::PutNumber("Alpha FF Gain", m_shooterAlphaCoeff.kFF);
  frc::SmartDashboard::PutNumber("Alpha Max Output", m_shooterAlphaCoeff.kMaxOutput);
  frc::SmartDashboard::PutNumber("Alpha Min Output", m_shooterAlphaCoeff.kMinOutput);

  // Dipslay beta motor related PID constants
  frc::SmartDashboard::PutNumber("Beta P Gain", m_shooterBetaCoeff.kP);
  frc::SmartDashboard::PutNumber("Beta I Gain", m_shooterBetaCoeff.kI);
  frc::SmartDashboard::PutNumber("Beta D Gain", m_shooterBetaCoeff.kD);
  frc::SmartDashboard::PutNumber("Beta FF Gain", m_shooterBetaCoeff.kFF);
  frc::SmartDashboard::PutNumber("Beta Max Output", m_shooterBetaCoeff.kMaxOutput);
  frc::SmartDashboard::PutNumber("Beta Min Output", m_shooterBetaCoeff.kMinOutput);
}

/**
 * Read the dashboard for the shooter
 */
void Shooter::ReadDashboard()
{
  // Read RPM override value
  m_overridenRPM = frc::SmartDashboard::GetNumber("Overridden RPM", 0.0);

  // Read alpha motor PID constants
  m_shooterAlphaCoeff.kP   = frc::SmartDashboard::GetNumber("Alpha P Gain", 0.0);
  m_shooterAlphaCoeff.kI   = frc::SmartDashboard::GetNumber("Alpha I Gain", 0.0);
  m_shooterAlphaCoeff.kD   = frc::SmartDashboard::GetNumber("Alpha D Gain", 0.0);
  m_shooterAlphaCoeff.kFF  = frc::SmartDashboard::GetNumber("Alpha FF Gain", 0.0);
  m_shooterAlphaCoeff.kMinOutput = frc::SmartDashboard::GetNumber("Alpha Min Output", 0.0);
  m_shooterAlphaCoeff.kMaxOutput = frc::SmartDashboard::GetNumber("Alpha Max Output", 0.0);

  // Read beta motor PID constants
  m_shooterBetaCoeff.kP   = frc::SmartDashboard::GetNumber("Beta P Gain", 0.0);
  m_shooterBetaCoeff.kI   = frc::SmartDashboard::GetNumber("Beta I Gain", 0.0);
  m_shooterBetaCoeff.kD   = frc::SmartDashboard::GetNumber("Beta D Gain", 0.0);
  m_shooterBetaCoeff.kFF  = frc::SmartDashboard::GetNumber("Beta FF Gain", 0.0);
  m_shooterBetaCoeff.kMinOutput = frc::SmartDashboard::GetNumber("Beta Min Output", 0.0);
  m_shooterBetaCoeff.kMaxOutput = frc::SmartDashboard::GetNumber("Beta Max Output", 0.0);

}
