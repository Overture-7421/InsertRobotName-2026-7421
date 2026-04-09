// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "EjectCommand.h"

EjectCommand::EjectCommand(Intake* intake, Shooter* shooter, Processor* processor) {
  this->intake = intake;
  this->shooter = shooter;
  this->processor = processor;

	AddRequirements({ shooter, processor}); //Intake is crashing (Probably)

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

  intake->setRollersVoltage(IntakeConstants::IntakeOpen.rollers);
  processor->setProcessorVoltages(ProcessorConstants::Eject);

  if (!startedClosing && (now - enterTimestamp) > 0.3_s) {
    intake->intakeSlowModeFilter.Reset(intake->getIntakePosition());
    startedClosing = true;
  }
  

  if (startedClosing) {
    intake->setIntakeDistance(intake->intakeSlowModeFilter.Calculate(IntakeConstants::IntakeClose.intake));
  }


  if(intake->getIntakePosition() < IntakeConstants::RollersShouldNotBeMoving ){
    intake->setRollersVoltage(IntakeConstants::IntakeClose.rollers);
  }

}

void EjectCommand::End(bool interrupted) {
  inTargetState = false;
  startedClosing = false;
  shooter->Release();
  processor->setProcessorVoltages(ProcessorConstants::Stop);
  intake->setIntakeCmd({IntakeConstants::IntakeClose.rollers, intake->getIntakePosition()});
}

bool EjectCommand::IsFinished() {
  return false;
}
