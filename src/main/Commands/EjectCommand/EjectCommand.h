// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include "Subsystems/Shooter/Shooter.h"
#include "Subsystems/Intake/Intake.h"
#include "Subsystems/Processor/Processor.h"
#include "OvertureLib/Utils/UtilityFunctions/UtilityFunctions.h"


class EjectCommand
    : public frc2::CommandHelper<frc2::Command, EjectCommand> {
 public:
  
  EjectCommand(Intake* intake, Shooter* shooter, Processor* processor);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;


	Intake* intake = nullptr;
  Shooter* shooter = nullptr;
  Processor* processor = nullptr;

  bool inTargetState = false;
	bool startedClosing = false;
	units::time::second_t enterTimestamp = 0.0_s;
};
