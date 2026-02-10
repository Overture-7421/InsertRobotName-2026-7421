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
#include "OvertureLib/Math/TargetingWhileMoving/TargetingWhileMoving.h"
#include "LaunchConstants.h"

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
  LaunchCommand(Turret* turret, Shooter* shooter, Chassis* chassis, std::function<frc::Translation2d()> targetSupplier);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  Turret* turret = nullptr;
  Shooter* shooter = nullptr;
  Chassis* chassis = nullptr;

  std::function<frc::Translation2d()> targetSupplier;

  TargetingWhileMoving targetWhileMoving{ //Tiempo de Vuelo desde que sale la pieza hasta que llega al objetivo
    {
      {1.66_m, 0.0_s},
      {1.9_m, 0.0_s},
      {2.4_m, 0.0_s},
      {2.9_m, 0.0_s},
      {3.4_m, 0.0_s},
      {3.9_m, 0.0_s},
      {4.4_m, 0.0_s},
      {4.9_m, 0.0_s},
      {5.4_m, 0.0_s},
      {5.9_m, 0.0_s},
      {6.4_m, 0.0_s},
      {6.9_m, 0.0_s},
    }
  };


};
