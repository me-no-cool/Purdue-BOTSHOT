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

frc::Encoder shooterTopE(4, 5);
frc::Encoder shooterBottomE(0, 1);

frc::Encoder screwDriveE(2, 3);
frc::AnalogInput screwDriveBottom(0);
frc::AnalogInput screwDriveTop(1);

int counter = 0;
double sumPrev = 0;
int periodCount = 4;

void Robot::RobotInit()
{
  std::thread visionThread(&Robot::SkywalkerVisionThread, this);
  visionThread.detach();

  screwDriveE.SetDistancePerPulse(18.25 / 2287); //in per pulse for full stroke
  screwDriveE.SetReverseDirection(true);

  aimDistanceMacro = new AimDistanceMacro(this);
  aimDistanceMacro->SetDistance(distanceToHoop);

  flywheelControl = new KFFlywheelControl(this);
  flywheelControl->targetSpeed = targetSpeed;

  screwDriveControl = new ScrewDriveControl(this);
  screwDriveControl->targetPosition = targetScrewPosition;

  // AimDistanceMacro contains a flywheel control and screw drive control
  // so if you use it, do NOT use a seperate flywheel/screwdrive control
  if (useDistanceControl)
  {
    aimDistanceMacro->Run();
  }
  else
  {
    flywheelControl->Run();
    screwDriveControl->Run();
  }
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

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit()
{
}

void Robot::AutonomousPeriodic()
{
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic()
{
  GetAndSetUserDriveSpeeds();
  ManageLaunchControlMode();
  if (useDistanceControl)
    ManageAimDistanceMacro();
  else
    ManuallyManageFlywheelAndScrewdrive();

  if (driveControl.GetRawButton(3)) //Lift Motor Up - X Button
    liftMotor.Set(-.2);
  else if (driveControl.GetRawButton(2)) //Lift Motor Down - B Button
    liftMotor.Set(.2);
  else
    liftMotor.Set(0);

  intakeMotor.Set(driveControl.GetRawAxis(2) * -.3); //Intake Motor - LT
}

void Robot::TestPeriodic() {}

void Robot::SkywalkerVisionThread()
{
  cs::UsbCamera camera = frc::CameraServer::GetInstance()->StartAutomaticCapture();
  int pixelWidth = 640;
  int pixelHeight = 480;
  camera.SetResolution(pixelWidth, pixelHeight);
  cs::CvSink cvSink = frc::CameraServer::GetInstance()->GetVideo();
  cs::CvSource outputStreamStd = frc::CameraServer::GetInstance()->PutVideo("Targeting", pixelWidth, pixelHeight);
  cv::Mat source;
  cv::Mat withBar;
  cv::Mat output;
  cv::Point topOfBar(pixelWidth, pixelHeight / 2);
  cv::Point bottomOfBar(0, pixelHeight / 2);
  cv::Scalar targetingColor(0, 255, 0, 255);
  int lineWidth = 1; // must be >= 1
  double crossHairSize = 15;
  while (true)
  {

    cvSink.GrabFrame(source);
    if (!source.empty())
    {
      line(source, topOfBar, bottomOfBar, targetingColor, lineWidth);
      double xCrosshairCenter = (1 - crosshairProportionUpFeed) / 2 * bottomOfBar.x + (1 + crosshairProportionUpFeed) / 2 * topOfBar.x;
      double yCrosshairCenter = (1 - crosshairProportionUpFeed) / 2 * bottomOfBar.y + (1 + crosshairProportionUpFeed) / 2 * topOfBar.y;
      cv::Point crosshairTopLeft(xCrosshairCenter - crossHairSize, yCrosshairCenter + crossHairSize);
      cv::Point crosshairTopRight(xCrosshairCenter + crossHairSize, yCrosshairCenter + crossHairSize);
      cv::Point crosshairBottomLeft(xCrosshairCenter - crossHairSize, yCrosshairCenter - crossHairSize);
      cv::Point crosshairBottomRight(xCrosshairCenter + crossHairSize, yCrosshairCenter - crossHairSize);
      line(source, crosshairTopLeft, crosshairBottomRight, targetingColor, lineWidth);
      line(source, crosshairTopRight, crosshairBottomLeft, targetingColor, lineWidth);
      outputStreamStd.PutFrame(source);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

void Robot::ManageLaunchControlMode()
{
  if (driveControl.GetRawButton(5))
  { //Toggle Distance Control -- LB
    // debouncing code copy/pasted from Matt
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
  }

  // Toggle whether or not the flywheel should be on in distance mode
  
  if (driveControl.GetRawButton(6))
  { //Toggle Distance Control -- LB
    // debouncing code copy/pasted from Matt
    toggleFlywheelEnable = !toggleFlywheelEnable;
    if (toggleFlywheelEnable == true && toggleFlywheelPress == true)
    {
      toggleFlywheelPress = false;
      aimDistanceMacro->flywheelControl->flywheelOn = true;
    }
    else if (toggleFlywheelEnable == false && toggleFlywheelPress == true)
    {
      toggleFlywheelPress = false;
      aimDistanceMacro->flywheelControl->flywheelOn = false;
    }
  }
  else
  {
    toggleFlywheelPress = true;
  }
}

void Robot::SetUseDistanceControl(bool useDistanceControl)
{
  if (this->useDistanceControl != useDistanceControl)
  {
    if (useDistanceControl)
    {
      this->useDistanceControl = true;
      this->aimDistanceMacro->Run();
      this->screwDriveControl->Terminate();
      this->flywheelControl->Terminate();
    }
    else
    {
      this->useDistanceControl = false;
      this->aimDistanceMacro->Terminate();
      this->screwDriveControl->Run();
      this->flywheelControl->Run();
    }
  }
}

void Robot::ManageAimDistanceMacro()
{
  // INTEGRATE VISION SYSTEM DISTANCE TO HOOP HERE @TODO
  double maxDistance = 25; // fe
  if (driveControl.GetRawButton(4))
  {                                     //Screw Drive Motor
    crosshairProportionUpFeed += 0.001; // unitless, -1 to 1
    crosshairProportionUpFeed = std::min(1.0, crosshairProportionUpFeed);
    distanceToHoop = AimDistanceMacro::calculateDistanceToHoop(crosshairProportionUpFeed);
    aimDistanceMacro->SetDistance(distanceToHoop); // give control input
  }
  else if (driveControl.GetRawButton(1))
  {
    crosshairProportionUpFeed -= 0.001; // unitless
    crosshairProportionUpFeed = std::max(-1.0, crosshairProportionUpFeed);
    distanceToHoop = AimDistanceMacro::calculateDistanceToHoop(crosshairProportionUpFeed);
    aimDistanceMacro->SetDistance(distanceToHoop);
  }

  aimDistanceMacro->Update();
  std::cout << "Dist: " << distanceToHoop
            << "\t\tVelSet: " << aimDistanceMacro->flywheelControl->targetSpeed
            << "\t\tVelAct: " << shooterTopE.GetRate() / 6634.0
            << "\t\tScrSet: " << aimDistanceMacro->screwDriveControl->targetPosition
            << "\t\tScrAct: " << screwDriveE.GetDistance()
            << std::endl;
}

void Robot::ManuallyManageFlywheelAndScrewdrive()
{
  targetSpeed = GetUserFlywheelSpeed(targetSpeed);
  flywheelControl->targetSpeed = targetSpeed;
  flywheelControl->Update();
  //OutputDebugFlywheelSpeeds(targetSpeed);
  if (driveControl.GetRawButton(1))
  {                             //Screw Drive Motor
    targetScrewPosition -= 0.1; // in
    screwDriveControl->Seek(targetScrewPosition);
  }
  else if (driveControl.GetRawButton(4))
  {
    targetScrewPosition += 0.1; // in
    screwDriveControl->Seek(targetScrewPosition);
  }
  //std::cout << "targetScrewPos: " << targetScrewPosition;
  screwDriveControl->Update();
  std::cout << "\t\tVelSet: " << flywheelControl->targetSpeed
            << "\t\tVelAct: " << shooterTopE.GetRate() / 6634.0
            << "\t\tScrSet: " << screwDriveControl->targetPosition
            << "\t\tScrAct: " << screwDriveE.GetDistance()
            << std::endl;
}

void Robot::GetAndSetUserDriveSpeeds()
{
  double moveValue = driveControl.GetRawAxis(1);
  double rotateValue = driveControl.GetRawAxis(0);
  moveValue = moveValue * moveValue * moveValue;         // cubic filter for deadband,
  rotateValue = rotateValue * rotateValue * rotateValue; // finer smallscale precision
  moveValue *= driveSpeedReduction;
  rotateValue *= driveSpeedReduction;
  drivetrain.ArcadeDrive(moveValue, rotateValue); //Call this to control the drive motors
}

float Robot::GetUserFlywheelSpeed(float currentTargetSpeed)
{
  float targetSpeed = currentTargetSpeed;
  //Uses Select and Start Buttons to Adjust the Speed of the Shooter
  if (hold == 0)
  {
    if (driveControl.GetRawButton(7))
    {
      targetSpeed -= 0.1;
      hold = 1;
    }
    else if (driveControl.GetRawButton(8))
    {
      targetSpeed += 0.1;
      hold = 1;
    }
  }
  else
  {
    hold = 0;
  }

  if (driveControl.GetRawButton(6))
  { //Top Shooter Wheels - RB

    shootEnable = !shootEnable;

    if (shootEnable == true && shootPress == true)
    {
      shootPress = false;
      targetSpeed = 10.0f;
    }
    else if (shootEnable == false && shootPress == true)
    {
      shootPress = false;
      targetSpeed = 0;
    }
  }
  else
  {
    shootPress = true;
  }

  return targetSpeed;
}

void Robot::OutputDebugFlywheelSpeeds(float targetSpeed)
{
  if (counter % periodCount == 0)
  {
    std::cout << "D: " << targetSpeed << "\t ";
    sumPrev = 0;
  }
  double actualSpeed = shooterTopE.GetRate() / 6634.0;
  std::cout << "A: " << actualSpeed << "\t ";
  sumPrev += actualSpeed;
  if (counter % periodCount == periodCount - 1)
  {
    std::cout << "Avg: " << sumPrev / periodCount << std::endl;
  }
  counter++;
}

void KFFlywheelControl::Run()
{
  // initialize the slowed setpoint to the current speed to prevent abrupt changes
  slowedSetpoint = shooterTopE.GetRate() / 6634.0;
  this->currentState = CORRECTING;
}
void KFFlywheelControl::Terminate() { this->currentState = TERMINATED; }
void KFFlywheelControl::Update()
{
  double speedErrorTop;
  double speedErrorBot;
  switch (currentState)
  {
  case CORRECTING:
    speedErrorTop = (slowedSetpoint * 0.9f) - shooterTopE.GetRate() / 6634.0;
    robot->shooterTop1.Set(kf * (slowedSetpoint * 0.9f) + kp * speedErrorTop);
    robot->shooterTop2.Set(kf * (slowedSetpoint * 0.9f) + kp * speedErrorTop);
    speedErrorBot = slowedSetpoint - shooterBottomE.GetRate() / 6634;
    robot->shooterBottom1.Set(kf * slowedSetpoint + kp * speedErrorBot);
    robot->shooterBottom2.Set(kf * slowedSetpoint + kp * speedErrorBot);

    if (flywheelOn)
    {
      if (targetSpeed > slowedSetpoint)
      {
        slowedSetpoint = std::min(slowedSetpoint + setpointDeltaPerCycle, targetSpeed);
      }
      if (targetSpeed < slowedSetpoint)
      {
        slowedSetpoint = std::max(slowedSetpoint - setpointDeltaPerCycle, targetSpeed);
      }
    }
    else
    {
        slowedSetpoint = std::max(slowedSetpoint - setpointDeltaPerCycle, 0.0);
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
      robot->screwDrive.Set(-0.5);
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
    robot->screwDrive.Set(voltage);

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
  this->targetDistance = targetDistance;
  targetFlywheelSpeed = calculateFlywheelSpeed(targetDistance);
  targetScrewPosition = calculateScrewPosition(targetDistance);

  flywheelControl->targetSpeed = targetFlywheelSpeed;
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
    if (leftEntry.distance <= distance || distance <= rightEntry.distance)
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
    if (leftEntry.distance <= distance || distance <= rightEntry.distance)
    {
      //interpolate
      double proportionBetween = (distance - leftEntry.distance) / (rightEntry.distance - leftEntry.distance);
      double interpolatedValue = (1 - proportionBetween) * leftEntry.flywheelSpeed + proportionBetween * rightEntry.flywheelSpeed;
      return interpolatedValue;
    }
  }
  return 0; // should never reach here
}

double AimDistanceMacro::calculateDistanceToHoop(double crosshairProportionUpFeed)
{
  double maxDistance = 25;
  double hoopHeight = 100; // @TODO
  double fov = 0;//@TODO
  double camAngleOffset = 0; //@TODO
  
  double theta = camAngleOffset + fov/2 + atan(crosshairProportionUpFeed * tan(fov/2)); // BS, @TODO do this but actually


  return tan(theta) * hoopHeight;
}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
