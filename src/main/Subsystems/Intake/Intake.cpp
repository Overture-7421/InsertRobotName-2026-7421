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
	intakeSecondMotor.setRotorToSensorRatio(IntakeConstants::intakeRotorToSensor) ;
    intakeSecondMotor.setFusedCANCoder(IntakeConstants::intakeSecondCanCoderConfig().CanCoderId);


    intakeMotor.configureMotionMagic(IntakeConstants::IntakeCruiseVelocity, IntakeConstants::IntakeCruiseAcceleration, 0.0_tr_per_s_cu);
    intakeSecondMotor.configureMotionMagic(IntakeConstants::IntakeCruiseVelocity, IntakeConstants::IntakeCruiseAcceleration, 0.0_tr_per_s_cu);

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
}


frc2::CommandPtr Intake::setIntakeCmd(intakeValues targetPos){
    return frc2::FunctionalCommand(
        [this, targetPos] () {
            setIntakeAngle(targetPos.intake);
            setRollersVoltage(targetPos.rollers);
        },

        [] () {
        },

        [this] (bool interrupted){},

        [this, targetPos] {
            return (intakeReached(targetPos.intake));
        }
    ).ToPtr();
}

frc2::CommandPtr Intake::setPivotCmd(units::degree_t targetAngle){
    return frc2::FunctionalCommand(
        [this, targetAngle] () {
            setIntakeAngle(targetAngle);
        },

        [] () {
        },

        [this] (bool interrupted){},

        [this, targetAngle] {
            return (intakeReached(targetAngle));
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

frc2::CommandPtr Intake::setRollersCmd(units::volt_t targetVoltage){
    return frc2::FunctionalCommand(
        [this, targetVoltage] () {
            setRollersVoltage(targetVoltage);
        },

        [] () {
        },
        
        [] (bool interrupted){},

        [] {
            return true;
        }
    ).ToPtr();
}

// This method will be called once per scheduler run
void Intake::Periodic() {
}

void Intake::UpdateTelemetry() {
    frc::SmartDashboard::PutNumber("Intake/Current", intakeMotor.GetPosition().GetValue().value() * 360.0);

    
    double targetAngle = intakeMotor.GetClosedLoopReference().GetValue();
    frc::SmartDashboard::PutNumber("Intake/ErrorAngle", intakeMotor.GetClosedLoopError().GetValue());
    frc::SmartDashboard::PutNumber("Intake/TargetAngle", targetAngle);
    frc::SmartDashboard::PutBoolean("Intake/isIntakeAtAngle", intakeReached(units::degree_t(targetAngle)));
}