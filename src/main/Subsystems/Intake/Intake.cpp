// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Intake.h"
#include "IntakeConstants.h"
#include <frc2/command/FunctionalCommand.h>
#include <frc/smartdashboard/SmartDashboard.h>

Intake::Intake() {
    intakeMotor.setRotorToSensorRatio(intakeConstants::intakeRotorToSensor);
    intakeMotor.setFusedCANCoder(intakeConstants::intakeCanCoderConfig().CanCoderId);

    intakeMotor.configureMotionMagic(intakeConstants::IntakeCruiseVelocity, intakeConstants::IntakeCruiseAcceleration, 0.0_tr_per_s_cu);
}

void Intake::setRollersVoltage(units::volt_t targetVoltage){
    rollersMotor.SetVoltage(targetVoltage);
}

bool Intake::intakeReached(units::degree_t targetAngle){
    units::degree_t intakeError = targetAngle - intakeMotor.GetPosition().GetValue();
    return (units::math::abs(intakeError) < intakeConstants::IntakeRangeError);
}

void Intake::setIntakeAngle(units::degree_t targetAngle){
    frc::SmartDashboard::PutNumber("Intake Target Position:", targetAngle.value());
    intakeMotor.SetControl(intakeVoltage.WithPosition(targetAngle).WithEnableFOC(true));
}


frc2::CommandPtr Intake::setIntakePosition(intakeValues targetPos){
    return frc2::FunctionalCommand(
        [this, targetPos] () {
            setIntakeAngle(targetPos.intake);
        },

        [this, targetPos] () {
            setRollersVoltage(targetPos.rollers);
        },

        [this] (bool interrupted){},

        [this, targetPos] {
            return (intakeReached(targetPos.intake));
        }
    ).ToPtr();
}

// This method will be called once per scheduler run
void Intake::Periodic() {}
