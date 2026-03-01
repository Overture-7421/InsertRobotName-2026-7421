// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Intake.h"
#include "IntakeConstants.h"
#include <frc2/command/FunctionalCommand.h>
#include <frc/smartdashboard/SmartDashboard.h>

Intake::Intake() {
    intakeMotor.setRotorToSensorRatio(IntakeConstants::intakeRotorToSensor) ;
    intakeMotor.setFusedCANCoder(IntakeConstants::intakeCanCoderConfig().CanCoderId);


    intakeMotor.configureMotionMagic(IntakeConstants::IntakeCruiseVelocity, IntakeConstants::IntakeCruiseAcceleration, 0.0_tr_per_s_cu);
}

void Intake::setRollersVoltage(units::volt_t targetVoltage){
    rollersMotor.SetControl(rollersVoltage.WithOutput(targetVoltage).WithEnableFOC(true));
}
 
bool Intake::intakeReached(units::degree_t targetAngle){
    units::degree_t intakeError = targetAngle - intakeMotor.GetPosition().GetValue();
    return (units::math::abs(intakeError) < IntakeConstants::IntakeRangeError);
}

void Intake::setIntakeAngle(units::degree_t targetAngle){
    intakeMotor.SetControl(intakeVoltage.WithPosition(targetAngle).WithEnableFOC(true));
    frc::SmartDashboard::PutNumber("Intake/Target", targetAngle.value());

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

frc2::CommandPtr Intake::setIntakeCharacterization(units::degree_t angle, units::volt_t voltage){
    return frc2::cmd::RunOnce(
        [this, angle, voltage] {
            setIntakeAngle(angle);
            setRollersVoltage(voltage);
        }
    );
}

frc2::CommandPtr Intake::setRollersVoltageCommand(units::volt_t targetVoltage){
    return frc2::FunctionalCommand(
        [] () {
        },

        [this, targetVoltage] () {
            setRollersVoltage(targetVoltage);
        },
        
        [] (bool interrupted){},

        [] {
            return true;
        }
    ).ToPtr();
}

// This method will be called once per scheduler run
void Intake::Periodic() {
    frc::SmartDashboard::PutNumber("Intake/Current", intakeMotor.GetPosition().GetValue().value() * 360.0);
}
