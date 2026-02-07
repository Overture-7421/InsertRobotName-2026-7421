// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include "Subsystems/Turret/Turret.h"
#include "Subsystems/Shooter/Shooter.h"
#include "Subsystems/Chassis/Chassis.h"
#include "OvertureLib/Utils/UtilityFunctions/UtilityFunctions.h"
#include "pathplanner/lib/util/FlippingUtil.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class LaunchCommand
    : public frc2::CommandHelper<frc2::Command, LaunchCommand> {
 public:
  /* You should consider using the more terse Command factories API instead
   * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
   */
  LaunchCommand(Turret* turret, Shooter* shooter, Chassis* chassis, frc::Translation2d targetObjective);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  Turret* turret;
  Shooter* shooter;
  Chassis* chassis;

  frc::Translation2d targetObjective;
};
