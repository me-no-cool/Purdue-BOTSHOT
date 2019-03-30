/*----------------------------------------------------------------------------*/ /* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"
#include "frc/Encoder.h"

#include <iostream>
#include "math.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"

#include "frc/Talon.h"

#define B_LIFT_UP 3
#define B_LIFT_DOWN 2
#define B_BIAS_UP 4
#define B_BIAS_DOWN 1
#define B_FLYWHEEL_TOGGLE 6
#define B_DISTANCE_SET 5

frc::Encoder shooterTopE(4, 5);
frc::Encoder shooterBottomE(0, 1);

frc::Encoder screwDriveE(2, 3);
frc::AnalogInput screwDriveBottom(0);
frc::AnalogInput screwDriveTop(1);

frc::AnalogInput liftLimit(2);

int counter = 0;
double sumPrev = 0;
int periodCount = 4;

nt::NetworkTableEntry disEntry;

void Robot::RobotInit()
{
  //std::thread visionThread(&Robot::SkywalkerVisionThread, this);
  //visionThread.detach();
  nt::NetworkTableInstance inst = nt::NetworkTableInstance::GetDefault();
  auto table = inst.GetTable("lidar");
  disEntry = table->GetEntry("hoop_distance");

  const int kPeakCurrentAmps = 15; /* threshold to trigger current limit */
	const int kPeakTimeMs = 0; /* how long after Peak current to trigger current limit */
	const int kContinCurrentAmps = 10; /* hold current after limit is triggered */
	/* nonzero to block the config until success, zero to skip checking */
  const int kTimeoutMs = 30;

  (&shooterBottom1)->ConfigPeakCurrentLimit(kPeakCurrentAmps, kTimeoutMs);
  (&shooterBottom1)->ConfigPeakCurrentDuration(kPeakTimeMs, kTimeoutMs);
  (&shooterBottom1)->ConfigContinuousCurrentLimit(kContinCurrentAmps, kTimeoutMs);
  (&shooterBottom1)->EnableCurrentLimit(true);

  (&shooterBottom2)->ConfigPeakCurrentLimit(kPeakCurrentAmps, kTimeoutMs);
  (&shooterBottom2)->ConfigPeakCurrentDuration(kPeakTimeMs, kTimeoutMs);
  (&shooterBottom2)->ConfigContinuousCurrentLimit(kContinCurrentAmps, kTimeoutMs);
  (&shooterBottom2)->EnableCurrentLimit(true);

  (&shooterTop1)->ConfigPeakCurrentLimit(kPeakCurrentAmps, kTimeoutMs);
  (&shooterTop1)->ConfigPeakCurrentDuration(kPeakTimeMs, kTimeoutMs);
  (&shooterTop1)->ConfigContinuousCurrentLimit(kContinCurrentAmps, kTimeoutMs);
  (&shooterTop1)->EnableCurrentLimit(true);

  (&shooterTop2)->ConfigPeakCurrentLimit(kPeakCurrentAmps, kTimeoutMs);
  (&shooterTop2)->ConfigPeakCurrentDuration(kPeakTimeMs, kTimeoutMs);
  (&shooterTop2)->ConfigContinuousCurrentLimit(kContinCurrentAmps, kTimeoutMs);
  (&shooterTop2)->EnableCurrentLimit(true);

  screwDriveE.SetDistancePerPulse(18.25 / 2287); //in per pulse for full stroke
  screwDriveE.SetReverseDirection(true);

  aimDistanceMacro = new AimDistanceMacro(this);
  //SmartDashboard.Init();
  frc::SmartDashboard::PutNumber("Distance", disEntry.GetDouble(0));
  //SmartDashboard.putNumber("Distance Target", distanceToHoop);
  aimDistanceMacro->SetDistance(distanceToHoop + bias);

  // AimDistanceMacro contains a flywheel control and screw drive control
  // so if you use it, do NOT use a seperate flywheel/screwdrive control
  aimDistanceMacro->Run();
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic()
{
  GetAndSetUserDriveSpeeds();
  ManageLaunchControlMode();
  ManageAimDistanceMacro();

  frc::SmartDashboard::PutNumber("Distance", disEntry.GetDouble(0));

  if (driveControl.GetRawButton(B_LIFT_UP)) //Lift Motor Up - X Button
    liftMotor.Set(-.2);
  else if (driveControl.GetRawButton(B_LIFT_DOWN)) //Lift Motor Down - B Button
    liftMotor.Set(.2);
  else
    liftMotor.Set(0);
	
  if (liftLimit.GetVoltage() > 2.5)
    liftMotor.Set(.2);

  intakeMotor.Set(driveControl.GetRawAxis(2) * -.3); //Intake Motor - LT
}

void Robot::TestPeriodic() {}

void Robot::ManageLaunchControlMode()
{
  /*if (driveControl.GetRawButton(5))
  {
    toggleUseDistEnable = !toggleUseDistEnable;
    if (toggleUseDistEnable == true && toggleUseDistPress == true)
    {
      toggleUseDistPress = false;
      SetUseDistanceControl(true);
    }
    else if (toggleUseDistEnable == false && toggleUseDistPress == true)
    {
      toggleUseDistPress = false;
      SetUseDistanceControl(false);
    }
  }
  else
  {
    toggleUseDistPress = true;
  }*/

  // Toggle whether or not the flywheel should be on in distance mode
  
  if (driveControl.GetRawButton(B_FLYWHEEL_TOGGLE))
  {   
    if (toggleFlywheelPress == true)
    {
      toggleFlywheelPress = false;
	  toggleFlywheelEnable = !toggleFlywheelEnable;
      aimDistanceMacro->flywheelControl->flywheelOn = toggleFlywheelEnable;
    }
  }
  else
  {
    toggleFlywheelPress = true;
  }
  
  if (driveControl.GetRawButton(B_DISTANCE_SET))
  {
    distanceToHoop = disEntry.GetDouble(0); //Put lidar here
    bias = 0;
	  //SmartDashboard.putNumber("Distance Target", distanceToHoop);
    aimDistanceMacro->SetDistance(distanceToHoop + bias);
  }
}


void Robot::ManageAimDistanceMacro()
{
  // INTEGRATE VISION SYSTEM DISTANCE TO HOOP HERE @TODO
  double maxDistance = 25; // feet
  if (driveControl.GetRawButton(B_BIAS_UP) || driveControl.GetRawButton(7) || driveControl.GetRawButton(8))
  {                                     //Screw Drive Motor
    bias += 0.01;
	  //SmartDashboard.putNumber("Bias", bias);
    aimDistanceMacro->SetDistance(distanceToHoop + bias); // give control input
  }
  else if (driveControl.GetRawButton(B_BIAS_DOWN))
  {
	bias -= 0.01;
	  //SmartDashboard.putNumber("Bias", bias);
    aimDistanceMacro->SetDistance(distanceToHoop + bias);
  }

  aimDistanceMacro->Update();
  /*std::cout << "Dist: " << aimDistanceMacro->targetDistance
            << "\t\tVelSet: " << aimDistanceMacro->flywheelControl->targetSpeed
            << "\t\tVelActT: " << shooterTopE.GetRate() / 6634.0
            << "\t\tVelActB: " << shooterBottomE.GetRate() / 6634.0
            << "\t\tScrSet: " << aimDistanceMacro->screwDriveControl->targetPosition
            << "\t\tScrAct: " << screwDriveE.GetDistance()
            << std::endl;*/
}

void Robot::GetAndSetUserDriveSpeeds()
{
  double moveValue = driveControl.GetRawAxis(1);
  double rotateValue = driveControl.GetRawAxis(0);
  moveValue = moveValue * moveValue * moveValue;         // cubic filter for deadband,
  rotateValue = rotateValue * rotateValue * rotateValue; // finer smallscale precision
  moveValue *= driveSpeedReduction;
  rotateValue *= driveSpeedReduction * 0.80;
  drivetrain.ArcadeDrive(moveValue, rotateValue); //Call this to control the drive motors
}


void Robot::OutputDebugFlywheelSpeeds(float targetSpeed)
{
  if (counter % periodCount == 0)
  {
    //std::cout << "D: " << targetSpeed << "\t ";
    sumPrev = 0;
  }
  double actualSpeed = shooterTopE.GetRate() / 6634.0;
  //std::cout << "A: " << actualSpeed << "\t ";
  sumPrev += actualSpeed;
  if (counter % periodCount == periodCount - 1)
  {
    //std::cout << "Avg: " << sumPrev / periodCount << std::endl;
  }
  counter++;
}

void KFFlywheelControl::Run()
{
  // initialize the slowed setpoint to the current speed to prevent abrupt changes
  slowedSetpointTop = shooterTopE.GetRate() / 6634.0;
  slowedSetpointBottom = shooterBottomE.GetRate() / 6634.0;
  this->currentState = CORRECTING;
}
void KFFlywheelControl::Terminate() { this->currentState = TERMINATED; }
void KFFlywheelControl::Seek(double targetSpeed) {this->targetSpeed = targetSpeed; }
void KFFlywheelControl::Update()
{
  double speedErrorTop;
  double speedErrorBot;
  switch (currentState)
  {
  case CORRECTING:
    speedErrorTop = (slowedSetpointTop) - shooterTopE.GetRate() / 6634.0f;
	
	  frc::SmartDashboard::PutNumber("Flywheel Actual", shooterTopE.GetRate() / 6634.0f);
	
    robot->shooterTop1.Set(kf * (slowedSetpointTop) + kp * speedErrorTop);
    robot->shooterTop2.Set(kf * (slowedSetpointTop) + kp * speedErrorTop);

    speedErrorBot = slowedSetpointBottom - shooterBottomE.GetRate() / 6634.0f;
    robot->shooterBottom1.Set(kf * slowedSetpointBottom + kp * speedErrorBot);
    robot->shooterBottom2.Set(kf * slowedSetpointBottom + kp * speedErrorBot);

    if (flywheelOn)
    {
      if (targetSpeed > slowedSetpointTop)
      {
        slowedSetpointTop = std::min(slowedSetpointTop + setpointDeltaPerCycle, targetSpeed);
      }
      if (targetSpeed < slowedSetpointTop)
      {
        slowedSetpointTop = std::max(slowedSetpointTop - setpointDeltaPerCycle, targetSpeed);
      }
      if (targetSpeed > slowedSetpointBottom)
      {
        slowedSetpointBottom = std::min(slowedSetpointBottom + setpointDeltaPerCycle, targetSpeed);
      }
      if (targetSpeed < slowedSetpointBottom)
      {
        slowedSetpointBottom = std::max(slowedSetpointBottom - setpointDeltaPerCycle, targetSpeed);
      }
    }
    else
    {
        slowedSetpointTop = std::max(slowedSetpointTop - setpointDeltaPerCycle, 0.0);
        slowedSetpointBottom = std::max(slowedSetpointBottom - setpointDeltaPerCycle, 0.0);
    }
    
    break;
  }
}

void ScrewDriveControl::Run() { this->currentState = CALIBRATING; }
void ScrewDriveControl::Seek(double targetPosition)
{
  this->targetPosition = targetPosition;
  if (this->targetPosition != targetPosition)
  {
    this->targetPosition = targetPosition;
    prevTime = std::chrono::system_clock::now();
    cumulativeError = 0;
  }
  if (currentState != CALIBRATING)
    currentState = CORRECTING; // don't start seeking until calibration is done
}
void ScrewDriveControl::Calibrate() { this->currentState = CALIBRATING; }
void ScrewDriveControl::Terminate() { this->currentState = TERMINATED; }
void ScrewDriveControl::Update()
{
  double positionError;
  double voltage;
  switch (currentState)
  {
  case CALIBRATING:
    if (!IsAtBottom())
    {
      robot->screwDrive.Set(0.5);
    }
    else
    {
      robot->screwDrive.Set(0);
      screwDriveE.Reset();
      currentState = CORRECTING;
      prevTime = std::chrono::system_clock::now();
      cumulativeError = 0;
    }

    break;
  case CORRECTING:
    // PI control. Note: Accuracy increases when flywheel shaking breaks friction.
    positionError = (slowedPosition)-screwDriveE.GetDistance();

    auto currentTime = std::chrono::system_clock::now();
    double timeSinceLastUpdate = (currentTime - prevTime).count(); // in ns
    prevTime = currentTime;
    cumulativeError += positionError * timeSinceLastUpdate * growthConstant;
    cumulativeError = std::max(-maxCumErrorMag, std::min(maxCumErrorMag, cumulativeError));

    voltage = kp * positionError + ki * cumulativeError;
    voltage = std::min(voltageUpperClamp, std::max(voltageLowerClamp, voltage)); // clamping to +-0.2 V
    if (IsAtBottom() && voltage < 0)
      voltage = 0;
    if (IsAtTop() && voltage > 0)
      voltage = 0;
    robot->screwDrive.Set(-voltage);

    // prevent sharp input changes
    // jump by setpointDeltaPerCycle or to the value exactly, whichever is closer
    if (targetPosition > slowedPosition)
    {
      slowedPosition = std::min(slowedPosition + setpointDeltaPerCycle, targetPosition);
    }
    if (targetPosition < slowedPosition)
    {
      slowedPosition = std::max(slowedPosition - setpointDeltaPerCycle, targetPosition);
    }
	
	  //SmartDashboard.putNumber("Screw Actual", screwDriveE.GetDistance());
    //std::cout << "\t\ttarg: " << targetPosition << "\t\tactu: " << screwDriveE.GetDistance()
    //          << "\t\tdelta: " << timeSinceLastUpdate
    //          << "\t\tcume: " << cumulativeError << "\t\tvolt: " << voltage << std::endl;
    break;
  }
}

bool ScrewDriveControl::IsAtBottom()
{
  return (screwDriveBottom.GetVoltage() > 2.5); // roughly 0 for not, 5 for yes
}
bool ScrewDriveControl::IsAtTop()
{
  return (screwDriveTop.GetVoltage() > 2.5); // roughly 0 for not, 5 for yes
}

AimDistanceMacro::AimDistanceMacro(Robot *robot)
{
  this->robot = robot;
  targetFlywheelSpeed = calculateFlywheelSpeed(targetDistance);
  targetScrewPosition = calculateScrewPosition(targetDistance);
  flywheelControl = new KFFlywheelControl(robot);
  flywheelControl->targetSpeed = targetFlywheelSpeed;
  screwDriveControl = new ScrewDriveControl(robot);
  screwDriveControl->targetPosition = targetScrewPosition;
}
AimDistanceMacro::~AimDistanceMacro()
{
  flywheelControl->Terminate();
  screwDriveControl->Terminate();
  delete flywheelControl;
  delete screwDriveControl;
}
void AimDistanceMacro::SetDistance(double targetDistance)
{
  frc::SmartDashboard::PutNumber("Distance Target", targetDistance);
  this->targetDistance = targetDistance;
  targetFlywheelSpeed = calculateFlywheelSpeed(targetDistance);
  frc::SmartDashboard::PutNumber("Speed Target", targetFlywheelSpeed);
  targetScrewPosition = calculateScrewPosition(targetDistance);
  frc::SmartDashboard::PutNumber("Screw Target", targetScrewPosition);
  /*if (robot->driveControl.GetRawButton(B_BIAS_UP))
  {                                     //Screw Drive Motor
    screwDriveControl->targetPosition += 0.01;
    screwDriveControl->Seek(screwDriveControl->targetPosition);
  }
  if (robot->driveControl.GetRawButton(B_BIAS_DOWN))
  {                                     //Screw Drive Motor
    screwDriveControl->targetPosition -= 0.01;
    screwDriveControl->Seek(screwDriveControl->targetPosition);
  }
  if (robot->driveControl.GetRawButton(7))
  {                                     //Screw Drive Motor
    this->flywheelControl->targetSpeed -= 0.01;
  }
  if (robot->driveControl.GetRawButton(8))
  {                                     //Screw Drive Motor
   this->flywheelControl->targetSpeed += 0.01;
  }*/
  
  //SmartDashboard.putNumber("Screw Target", targetScrewPosition);
  //SmartDashboard.putNumber("Flywheel Target", targetFlywheelSpeed);

  this->flywheelControl->targetSpeed = targetFlywheelSpeed;
  screwDriveControl->Seek(targetScrewPosition);
}
void AimDistanceMacro::Run()
{
  currentState = AIMING;
  flywheelControl->Run();
  screwDriveControl->Run();
}
void AimDistanceMacro::Update()
{
  flywheelControl->Update();
  screwDriveControl->Update();
  if (currentState == AIMING || currentState == AIMED)
  {
    bool isWithinBounds = true;
    //@TODO check within bounds. May not be necessary.
    if (isWithinBounds)
      currentState = AIMED;
    else
      currentState = AIMING;
  }
}
bool AimDistanceMacro::IsAimed()
{
  return currentState == AIMED;
}
void AimDistanceMacro::Terminate()
{
  flywheelControl->Terminate();
  screwDriveControl->Terminate();
  currentState = TERMINATED;
}

double AimDistanceMacro::calculateScrewPosition(double distance)
{
  if (distanceLookupTable.size() <= 0)
    return 0; // lookup table is empty, so give a safe value
	
  if (distance < distanceLookupTable.at(0).distance)
    return distanceLookupTable.at(0).screwPosition; // below lowest value
	
  if (distance > distanceLookupTable.at(distanceLookupTable.size() - 1).distance)
    return distanceLookupTable.at(distanceLookupTable.size() - 1).screwPosition; // above highest value

  for (int i = 0; i < distanceLookupTable.size() - 1; i++)
  {
    distanceLookupTableEntry leftEntry = distanceLookupTable.at(i);
    distanceLookupTableEntry rightEntry = distanceLookupTable.at(i + 1);
    if (leftEntry.distance <= distance && distance <= rightEntry.distance)
    {
      //interpolate
      double proportionBetween = (distance - leftEntry.distance) / (rightEntry.distance - leftEntry.distance);
      double interpolatedValue = (1 - proportionBetween) * leftEntry.screwPosition + proportionBetween * rightEntry.screwPosition;
      return interpolatedValue;
    }
  }
  return 0; // should never reach here
}

double AimDistanceMacro::calculateFlywheelSpeed(double distance)
{
  if (distanceLookupTable.size() <= 0)
    return 0; // lookup table is empty, so give a safe value
	
  if (distance < distanceLookupTable.at(0).distance)
    return distanceLookupTable.at(0).flywheelSpeed; // below lowest value
	
  if (distance > distanceLookupTable.at(distanceLookupTable.size() - 1).distance)
    return distanceLookupTable.at(distanceLookupTable.size() - 1).flywheelSpeed; // above highest value

  for (int i = 0; i < distanceLookupTable.size() - 1; i++)
  {
    distanceLookupTableEntry leftEntry = distanceLookupTable.at(i);
    distanceLookupTableEntry rightEntry = distanceLookupTable.at(i + 1);
    if (leftEntry.distance <= distance && distance <= rightEntry.distance)
    {
      //interpolate
      double proportionBetween = (distance - leftEntry.distance) / (rightEntry.distance - leftEntry.distance);
      double interpolatedValue = (1 - proportionBetween) * leftEntry.flywheelSpeed + proportionBetween * rightEntry.flywheelSpeed;
      return interpolatedValue;
    }
  }
  return 0; // should never reach here
}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
