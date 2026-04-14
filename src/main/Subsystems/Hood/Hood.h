// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "HoodConstants.h"
#include "Constants.h"
#include <units/math.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/Commands.h>
#include <frc/smartdashboard/SmartDashboard.h>

class Hood : public frc2::SubsystemBase {
 public:
  Hood();


  void setHoodAngle(units::degree_t angle);
  units::degree_t getHoodAngle();
  bool isHoodAtAngle();
  frc2::CommandPtr setHoodAngleCommand(units::degree_t angle);

  void UpdateTelemetry();

  void Periodic() override;

 private:
  
  OverTalonFX hoodMotor{HoodConstants::MotorConfig(), robotConstants::rio};
 OverCANCoder hoodCANCoder{HoodConstants::CANCoderConfig(), robotConstants::rio};

  ctre::phoenix6::controls::MotionMagicVoltage hoodVoltageRequest{0.0_tr};
  units::degree_t targetAngle = 0_deg;

};
