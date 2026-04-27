// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "EjectCommand.h"

EjectCommand::EjectCommand(Intake* intake, Shooter* shooter, Processor* processor) {
	this->intake = intake;
	this->shooter = shooter;
	this->processor = processor;

	AddRequirements({ processor }); //Intake is crashing (Probably)

}

void EjectCommand::Initialize() {
	intake->intakeSlowModeFilter.Reset(intake->getIntakePosition());
}

void EjectCommand::Execute() {

	const units::time::second_t now = frc::Timer::GetFPGATimestamp();

	if (!inTargetState) {
		inTargetState = true;
		startedClosing = false;
		enterTimestamp = now;
		shooter->Hold();
	}

	processor->setProcessorVoltages(ProcessorConstants::Eject);

	if (!startedClosing && (now - enterTimestamp) > 0.75_s) {
		intake->setRollersVoltage(IntakeConstants::IntakeOpen.rollers);
	}

	if (!startedClosing && (now - enterTimestamp) > 1.40_s) {
		intake->intakeSlowModeFilter.Reset(intake->getIntakePosition());
		startedClosing = true;
	}


	if (startedClosing) {
		intake->setIntakeDistance(intake->intakeSlowModeFilter.Calculate(IntakeConstants::IntakeClose.intake));
	}

}

void EjectCommand::End(bool interrupted) {
	inTargetState = false;
	startedClosing = false;
	shooter->Release();
	processor->setProcessorVoltages(ProcessorConstants::Stop);
	intake->setRollersVoltage(IntakeConstants::IntakeClose.rollers);

}

bool EjectCommand::IsFinished() {
	return false;
}
