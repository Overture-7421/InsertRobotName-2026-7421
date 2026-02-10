// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <frc/geometry/Translation2d.h>
#include <algorithm>
#include "OvertureLib/Math/InterpolatingTable/InterpolatingTable.h"
#include <units/length.h>
#include <units/time.h>
#include <units/angle.h>


namespace LaunchConstants {

  constexpr static const frc::Translation2d HubPose = {0.0_m, 0.0_m}; //Poner
  constexpr static const frc::Translation2d LeftPass = {0.0_m, 0.0_m}; //Poner
  constexpr static const frc::Translation2d RightPass = {0.0_m, 0.0_m}; //Poner
  constexpr static const frc::Translation2d CenterPass = {0.0_m, 0.0_m}; //Poner

  static const InterpolatingTable<units::meter_t, units::degree_t> DistanceToHoodAngle{
      {
        {1.66_m, 0.0_deg},
        {1.9_m, 0.0_deg},
        {2.4_m, 0.0_deg},
        {2.9_m, 0.0_deg},
        {3.4_m, 0.0_deg},
        {3.9_m, 0.0_deg},
        {4.4_m, 0.0_deg},
        {4.9_m, 0.0_deg},
        {5.4_m, 0.0_deg},
        {5.9_m, 0.0_deg},
        {6.4_m, 0.0_deg},
        {6.9_m, 0.0_deg}
      }
  }; 


  static const InterpolatingTable<units::meter_t, double> DistanceToShooter{
    {
        {0.1_m, 0},
        {0.5_m, 0},
        {1.0_m, 0},
        {1.5_m, 0},
        {2.0_m, 0},
        {2.5_m, 0},
        {3.0_m, 0},
        {3.5_m, 0},
        {4.0_m, 0},
        {4.5_m, 0},
    }
  };

};
