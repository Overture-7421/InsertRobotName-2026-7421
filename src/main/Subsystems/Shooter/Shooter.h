// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "ShooterConstants.h"
#include "Constants.h"
#include <ctre/phoenix6/controls/VelocityVoltage.hpp>
#include <units/math.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/Commands.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "ShooterState.h"
#include <wpi/circular_buffer.h>

class Shooter : public frc2::SubsystemBase {
public:

  Shooter();
  void setObjectiveVelocity(units::turns_per_second_t velocity);
  units::turns_per_second_t getShooterVelocity();
  bool isShooterAtVelocity();
  const ShooterState& getState();
  /**
  * Whether to keep Holding state, even though target may be off. This is useful when continously shooting fuel, as they slow down the flywheel.
  * "Hold" instructs the Shooter to keep Holding PID Slot (No PID gains, only kS,kV,kA) once we initially reach Holding state.
  * "Release" allows Holding state to transition to WindUp if we are no longer at the target. Windup returns to "normal" slot (Both PID gains and kS,kV,kA).
  */
  void Hold();
  void Release();

  frc2::CommandPtr setShooterVelocityCmd(units::turns_per_second_t velocity);
  void UpdateTelemetry();
  void Periodic() override;

private:

  OverTalonFX shooterLeftUpMotor{ShooterConstants::ShooterLeftUpConfig(), robotConstants::rio};
  OverTalonFX shooterLeftDownMotor{ShooterConstants::ShooterLeftDownConfig(), robotConstants::rio};
  OverTalonFX shooterRightUpMotor{ShooterConstants::ShooterRightUpConfig(), robotConstants::rio};
  OverTalonFX shooterRightDownMotor{ShooterConstants::ShooterRightDownConfig(), robotConstants::rio};

  ctre::phoenix6::controls::MotionMagicVelocityVoltage shooterVoltageRequest{0.0_tps};
  units::turns_per_second_t targetVelocity = 0_tps;

  units::second_t lastTimeOnTarget = 0_s;
  ShooterState state = ShooterState::WindUp;
  bool shouldHold = false;

  wpi::circular_buffer<double> kVEstimator {20};
  double averagekV = 0;
  int currentPIDSlot = 0;
};
