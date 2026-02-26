// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "LedsWinAuto.h"

frc2::CommandPtr LedsWinAuto(LedsManager* leds){
    return frc2::cmd::Sequence(
        StaticEffect(leds, "all", {0, 255, 0}). ToPtr().IgnoringDisable(true),
        frc2::cmd::Wait(7.0_s),
        BlinkEffect(leds, "all", {0, 255, 0}, 0.2_s).ToPtr().IgnoringDisable(true),
        frc2::cmd::Wait(3.0_s),
        StaticEffect(leds, "all", {107, 53, 170}). ToPtr().IgnoringDisable(true),
        frc2::cmd::Wait(22.0_s),
        BlinkEffect(leds, "all", {107, 53, 170}, 0.2_s).ToPtr().IgnoringDisable(true),
        frc2::cmd::Wait(3.0_s),
        StaticEffect(leds, "all", {0, 255, 0}). ToPtr().IgnoringDisable(true),
        frc2::cmd::Wait(22.0_s),
        BlinkEffect(leds, "all", {0, 255, 0}, 0.2_s).ToPtr().IgnoringDisable(true),
        frc2::cmd::Wait(3.0_s),
        StaticEffect(leds, "all", {107, 53, 170}). ToPtr().IgnoringDisable(true),
        frc2::cmd::Wait(22.0_s),
        BlinkEffect(leds, "all", {107, 53, 170}, 0.2_s).ToPtr().IgnoringDisable(true),
        frc2::cmd::Wait(3.0_s),
        StaticEffect(leds, "all", {0, 255, 0}). ToPtr().IgnoringDisable(true),
        frc2::cmd::Wait(50.0_s),
        BlinkEffect(leds, "all", {0, 255, 0}, 0.2_s).ToPtr().IgnoringDisable(true)
    );
}