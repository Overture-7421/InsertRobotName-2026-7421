// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Intake.h"
#include "IntakeConstants.h"
#include <frc2/command/FunctionalCommand.h>
#include <frc/smartdashboard/SmartDashboard.h>

Intake::Intake() {

	leftMotor.setFollow(rightMotor .GetDeviceID(), true);

	rightMotor.setRotorToSensorRatio(IntakeConstants::intakeRotorToSensor);
	rightMotor.setFusedCANCoder(IntakeConstants::RightCanCoderConfig().CanCoderId);
	leftMotor.setRotorToSensorRatio(IntakeConstants::intakeRotorToSensor);
	leftMotor.setFusedCANCoder(IntakeConstants::LeftCanCoderConfig().CanCoderId);


	rightMotor.configureMotionMagic(IntakeConstants::IntakeCruiseVelocity, IntakeConstants::IntakeCruiseAcceleration, 0.0_tr_per_s_cu);
	leftMotor.configureMotionMagic(IntakeConstants::IntakeCruiseVelocity, IntakeConstants::IntakeCruiseAcceleration, 0.0_tr_per_s_cu);

}

void Intake::setRollersVoltage(units::volt_t targetVoltage) {
	rollersMotor.SetControl(rollersVoltage.WithOutput(targetVoltage).WithEnableFOC(true));
}

units::degree_t Intake::transformCentimetersToDegrees(units::centimeter_t distance){
	return units::degree_t((distance.value()/IntakeConstants::PinionRadius.value())*180/3.14);	
}

units::centimeter_t Intake::transformDegreesToCentimeters(units::degree_t angle){
	return units::centimeter_t((angle.value()*3.14/180)*IntakeConstants::PinionRadius.value());
}

bool Intake::intakeReached(units::centimeter_t targetDistance) {
	//units::degree_t targetAngle = units::degree_t(targetDistance.value()/IntakeConstants::PinionRadius.value()*180/3.14);
	units::degree_t targetAngle = transformCentimetersToDegrees(targetDistance);

	//units::centimeter_t intakeError = units::centimeter_t((targetAngle.value() - rightMotor.GetPosition().GetValue().value())*2*3.14*IntakeConstants::PinionRadius/360);
	units::centimeter_t intakeError = units::centimeter_t(targetAngle.value() - transformDegreesToCentimeters(rightMotor.GetPosition().GetValue()).value());
	return (units::math::abs(intakeError) < IntakeConstants::IntakeRangeError);
}

void Intake::setIntakeDistance(units::centimeter_t targetDistance) {
	units::degree_t targetAngle = units::degree_t(targetDistance.value()/IntakeConstants::PinionRadius.value()*180/3.14);
	rightMotor.SetControl(intakeVoltage.WithPosition(targetAngle).WithEnableFOC(true));
	leftMotor.SetControl(intakeVoltage.WithPosition(targetAngle).WithEnableFOC(true));
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
	}
	).ToPtr();
}

frc2::CommandPtr Intake::setPivotCmd(units::centimeter_t targetDistance) {
	return frc2::FunctionalCommand(
		[this, targetDistance]() {
		setIntakeDistance(targetDistance);
	},

		[]() {
	},

	[this](bool interrupted) {},

	[this, targetDistance] {
		return (intakeReached(targetDistance));
	}
	).ToPtr();
}

frc2::CommandPtr Intake::setIntakeCharacterization(units::centimeter_t distance, units::volt_t voltage) {
	return frc2::cmd::RunOnce(
		[this, distance, voltage] {
		setIntakeDistance(distance);
		setRollersVoltage(voltage);
	}
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
	}
	).ToPtr();
}

// This method will be called once per scheduler run
void Intake::Periodic() {}

void Intake::UpdateTelemetry() {
	frc::SmartDashboard::PutNumber("Intake/Current", rightMotor.GetPosition().GetValue().value() * 360.0);


	double targetDistance = rightMotor.GetClosedLoopReference().GetValue();
	frc::SmartDashboard::PutNumber("Intake/ErrorAngle", rightMotor.GetClosedLoopError().GetValue());
	frc::SmartDashboard::PutNumber("Intake/TargetAngle", targetDistance);
	frc::SmartDashboard::PutBoolean("Intake/isIntakeAtAngle", intakeReached(units::centimeter_t(targetDistance)));
}