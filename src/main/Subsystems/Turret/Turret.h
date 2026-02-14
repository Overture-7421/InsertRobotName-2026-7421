// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "TurretConstants.h"
#include "Constants.h"
#include <ctre/phoenix6/controls/MotionMagicVoltage.hpp>
#include <units/math.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/Commands.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Pose2d.h>
#include <frc/MathUtil.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/controller/ProfiledPIDController.h>
#include "Subsystems/Chassis/Chassis.h"

class Turret : public frc2::SubsystemBase {
 public:
  Turret(Chassis* chassis);

  void setTargetAngle(units::degree_t turretTarget);
  units::degree_t convertToClosestBoundedTurretAngleDegrees(units::degree_t targetAngleDegrees);
  units::degree_t calculateTurretAngleFromCANCoderDegrees();

  frc::Rotation2d GetTurretAimingParameterFromRobotPose(const frc::Pose2d& robotPose, const frc::Translation2d& targetPosition);

  void AimAtFieldPosition(const frc::Pose2d& robotPose, const frc::Translation2d& targetPosition);

  bool isAimAtFieldPosition(units::degree_t setPoint);

  frc2::CommandPtr TestCommand(units::degree_t setPoint);
  
  void Periodic() override;

 private:
  bool enableChassisComp = false;
  
  OverTalonFX turretMotor{TurretConstants::TurretConfig(), robotConstants::rio};
  OverCANCoder turret1CANCoder{TurretConstants::Turret1CANConfig(), robotConstants::rio};
  OverCANCoder turret2CANCoder{TurretConstants::Turret2CANConfig(), robotConstants::rio};

  // ctre::phoenix6::controls::VoltageOut turretVoltageRequest{0.0_V};
  ctre::phoenix6::controls::PositionVoltage turretVoltageRequest{0.0_tr};


    // frc::ProfiledPIDController<units::degree> turretPID {0.1, 0.0, 0.0, {TurretConstants::TurretVelocity,
    //         TurretConstants::TurretAcceleration}, RobotConstants::LoopTime};

  Chassis* chassis;

};
