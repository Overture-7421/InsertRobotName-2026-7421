// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/MathUtil.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/Servo.h>
#include "Robot.h"
#include "Subsystems/Climber/ClimberConstants.h"

class Climber : public frc2::SubsystemBase {
    public:
    Climber();

    frc2::CommandPtr SetPosition(units::length::meter_t target);

    void moveClimber(units::length::meter_t target);
    
    bool isFinished(units::length::meter_t TargetValue);

    units::length::meter_t getPosition();


    private:
        OverTalonFX leftClimberMotor {ClimberConstants::LeftConfig(), robotConstants::rio};
        OverTalonFX rightClimberMotor {ClimberConstants::RightConfig(), robotConstants::rio};

        ctre::phoenix6::controls::MotionMagicVoltage climberVoltage {0_tr};
};