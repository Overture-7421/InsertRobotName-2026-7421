// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "EjectCommand.h"

EjectCommand::EjectCommand(Processor* processor, Turret* turret, Intake* intake) {
	// Use addRequirements() here to declare subsystem dependencies.
	this->processor = processor;
	this->turret = turret;
	this->intake = intake;
	AddRequirements({ processor });
}

// Called when the command is initially scheduled.
void EjectCommand::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void EjectCommand::Execute() {

	if (turret->isMotorAtPosition()) {
		processor->setSpindexerPasserVoltage(ProcessorConstants::Eject);
		intake->setRollersVoltage(IntakeConstants::RollersEject);
	} else {
		processor->setSpindexerPasserVoltage(ProcessorConstants::StopProcessor);
		intake->setRollersVoltage(IntakeConstants::RollersStop);
	}

}

// Called once the command ends or is interrupted.
void EjectCommand::End(bool interrupted) {}

// Returns true when the command should end.
bool EjectCommand::IsFinished() {
	return false;
}
