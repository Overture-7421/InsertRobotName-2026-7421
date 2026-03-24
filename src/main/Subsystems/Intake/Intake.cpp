// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Intake.h"
#include "IntakeConstants.h"
#include <frc2/command/FunctionalCommand.h>
#include <frc/smartdashboard/SmartDashboard.h>

Intake::Intake() {

	pivotLeftMotor.setFollow(pivotRightMotor.GetDeviceID(), true);

	pivotRightMotor.setRotorToSensorRatio(IntakeConstants::intakeRotorToSensor);
	pivotRightMotor.setFusedCANCoder(IntakeConstants::pivotRightCanCoderConfig().CanCoderId);

	pivotRightMotor.configureMotionMagic(IntakeConstants::IntakeCruiseVelocity, IntakeConstants::IntakeCruiseAcceleration, 0.0_tr_per_s_cu);

}

void Intake::setRollersVoltage(units::volt_t targetVoltage) {
	rollersMotor.SetControl(rollersVoltage.WithOutput(targetVoltage).WithEnableFOC(true));
}

units::turn_t Intake::transformMetersToTurns(units::meter_t distance){
	return units::turn_t(distance.value()/IntakeConstants::PinionDiameter.value()*M_PI);	
}

units::meter_t Intake::transformTurnsToMeters(units::turn_t angle){
	return units::meter_t (angle.value()*IntakeConstants::PinionDiameter.value()*M_PI);
}

bool Intake::intakeReached(units::meter_t targetDistance) {
	//units::degree_t targetAngle = units::degree_t(targetDistance.value()/IntakeConstants::PinionRadius.value()*180/3.14);
	units::turn_t targetAngle = transformMetersToTurns(targetDistance);

	//units::meter_t intakeError = units::meter_t((targetAngle.value() - pivotRightMotor.GetPosition().GetValue().value())*2*3.14*IntakeConstants::PinionRadius/360);
	units::meter_t intakeError = units::meter_t(targetAngle.value() - transformTurnsToMeters(pivotRightMotor.GetPosition().GetValue()).value());
	return (units::math::abs(intakeError) < IntakeConstants::IntakeRangeError);
}

void Intake::setIntakeDistance(units::meter_t targetDistance) {
	units::turn_t targetAngle = transformMetersToTurns(targetDistance);
	pivotRightMotor.SetControl(intakeVoltage.WithPosition(targetAngle).WithEnableFOC(true));
}


frc2::CommandPtr Intake::setIntakeCmd(intakeValues targetPos) {
	return frc2::FunctionalCommand(
		[this, targetPos]() {
		setIntakeDistance(targetPos.intake);
		setRollersVoltage(targetPos.rollers);
	},

		[]() {
	},

	[this](bool interrupted) {},

	[this, targetPos] {
		return (intakeReached(targetPos.intake));
	}, {this}
	).ToPtr();
}

frc2::CommandPtr Intake::setPivotCmd(units::meter_t targetDistance) {
	return frc2::FunctionalCommand(
		[this, targetDistance]() {
		setIntakeDistance(targetDistance);
	},

		[]() {
	},

	[this](bool interrupted) {},

	[this, targetDistance] {
		return (intakeReached(targetDistance));
	}, {this}
	).ToPtr();}

frc2::CommandPtr Intake::setIntakeCharacterization(units::meter_t distance, units::volt_t voltage) {
	return frc2::cmd::RunOnce(
		[this, distance, voltage] {
		setIntakeDistance(distance);
		setRollersVoltage(voltage);
	}, {this}
	);
}

frc2::CommandPtr Intake::setRollersCmd(units::volt_t targetVoltage) {
	return frc2::FunctionalCommand(
		[this, targetVoltage]() {
		setRollersVoltage(targetVoltage);
	},

		[]() {
	},

	[](bool interrupted) {},

	[] {
		return true;
	}, {this}
	).ToPtr();
}

// This method will be called once per scheduler run
void Intake::Periodic() {}

void Intake::UpdateTelemetry() {
	frc::SmartDashboard::PutNumber("Intake/Current", pivotRightMotor.GetPosition().GetValue().value() * 360.0);


	double targetDistance = pivotRightMotor.GetClosedLoopReference().GetValue();
	frc::SmartDashboard::PutNumber("Intake/ErrorAngle", pivotRightMotor.GetClosedLoopError().GetValue());
	frc::SmartDashboard::PutNumber("Intake/TargetAngle", targetDistance);
	frc::SmartDashboard::PutBoolean("Intake/isIntakeAtAngle", intakeReached(units::meter_t(targetDistance)));
}