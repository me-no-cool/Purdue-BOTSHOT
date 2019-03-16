/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <string>

#include <IterativeRobot.h>
#include <SmartDashboard/SendableChooser.h>
#include "WPILib.h"
#include "CameraServer.h"
#include "ctre/Phoenix.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

class KFFlywheelControl;
class ScrewDriveControl;
class AimDistanceMacro;

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;

  void SkywalkerVisionThread();

  void GetAndSetUserDriveSpeeds();
  void ManageLaunchControlMode();
  void ManageAimDistanceMacro();
  void ManuallyManageFlywheelAndScrewdrive();
  float GetUserFlywheelSpeed(float currentTargetSpeed);
  void OutputDebugFlywheelSpeeds(float targetSpeed);
  void SetUseDistanceControl(bool useDistanceControl);

  float targetSpeed = 0;
  float targetScrewPosition = 5;
  float distanceToHoop = 12;
  double crosshairProportionUpFeed = 0.5;

  int hold = 0;
  bool shootEnable = false;
  bool shootPress = false;
  bool toggleUseDistEnable = false;
  bool toggleUseDistPress = false;
  bool toggleFlywheelEnable = false;
  bool toggleFlywheelPress = false;

  bool useDistanceControl = true;
  
  double driveSpeedReduction = 0.6;
 
 //private:
  WPI_VictorSPX leftMotorA{5};
  WPI_VictorSPX leftMotorB{0};
  WPI_VictorSPX rightMotorA{10};
  WPI_VictorSPX rightMotorB{11};
  WPI_TalonSRX screwDrive{6};
  WPI_TalonSRX shooterBottom1{8};
  WPI_TalonSRX shooterBottom2{9};
  WPI_TalonSRX shooterTop1{4};
  WPI_TalonSRX shooterTop2{3};
  WPI_VictorSPX liftMotor{2};
  WPI_VictorSPX intakeMotor{1};

  AimDistanceMacro* aimDistanceMacro;
  KFFlywheelControl* flywheelControl;
  ScrewDriveControl* screwDriveControl;

  frc::SpeedControllerGroup leftDrive{leftMotorA, leftMotorB};
  frc::SpeedControllerGroup rightDrive{rightMotorA, rightMotorB};

  frc::DifferentialDrive drivetrain{leftDrive, rightDrive};

  frc::Joystick driveControl{0};
  
};

class KFFlywheelControl
{
  public:  
    enum State {
      NEW,
      CORRECTING,
      TERMINATED
    };
    KFFlywheelControl(Robot* robot) {this->robot = robot; }
    void Run();
    void Update();
    void Terminate();

    double targetSpeed;
    bool flywheelOn = true;
  private:
    Robot* robot;
    State currentState = NEW;
    double slowedSetpoint = 0;
    double kf = 0.03;
    double kp = 0.02;
    double backspinConst = 0.9;
    double setpointDeltaPerCycle = 0.1;

};

class ScrewDriveControl
{
  public:  
    enum State {
      NEW,
      CALIBRATING,
      CORRECTING,
      TERMINATED
    };
    ScrewDriveControl(Robot* robot) {this->robot = robot;}
    void Run();
    void Update();
    void Calibrate();
    void Seek(double targetPosition); // 0 <= position <= 18.25 in
    void Terminate();

    bool IsAtBottom();
    bool IsAtTop();
    
    double targetPosition;
  private:
    Robot* robot;
    long long startTime;
    State currentState = NEW;
    double slowedPosition = 0;
    double setpointDeltaPerCycle = 0.1;
    double voltageUpperClamp = 1;
    double voltageLowerClamp = -1;
    double kp = 1;
    double ki = 0.05;
    double cumulativeError = 0;
    double growthConstant = 1.0 / 10000000; // controls how quickly error accumulates
    double maxCumErrorMag = 20;
    std::chrono::time_point<std::chrono::system_clock> prevTime;

};

class AimDistanceMacro
{
  public:  
    enum State {
      NEW,
      AIMING,
      AIMED,
      TERMINATED
    };
    AimDistanceMacro(Robot* robot);
    ~AimDistanceMacro();
    void Run();
    void Update();
    bool IsAimed();
    void Terminate();

    void SetDistance(double distance);
    double calculateScrewPosition(double distance);
    double calculateFlywheelSpeed(double distance);
    static double calculateDistanceToHoop(double crosshairProportionUpFeed);

    KFFlywheelControl* flywheelControl;
    ScrewDriveControl* screwDriveControl;
  private:
    Robot* robot;
    State currentState = NEW;
    double targetDistance = 0;
    double targetScrewPosition;
    double targetFlywheelSpeed;
    double screwThreshold = 0.1; //in
    double flywheelSpeedThreshold = 1.5; // rps

    struct distanceLookupTableEntry {
      double distance; // ft
      double screwPosition; // in
      double flywheelSpeed; // rps
    };
    std::vector< distanceLookupTableEntry> distanceLookupTable =
    {
      // Note: Entries should be sorted by distance,
      // and there should be NO DUPLICATES
      { 0, 3, 15},
      { 3, 6.875, 15},
      { 6, 11.125, 15},
      { 9, 13.125, 16},
      { 12, 18, 16},
      { 15, 15.375, 19},
      { 18, 19.5, 19},
      { 21, 20, 20}
    };

};
