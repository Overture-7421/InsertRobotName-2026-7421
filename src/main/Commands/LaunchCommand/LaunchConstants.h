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
#include <units/angular_velocity.h>


namespace LaunchConstants {

  constexpr static const frc::Translation2d HubPose = {4.6115986_m, 4.0213534_m}; //Poner
  constexpr static const frc::Translation2d LeftPass = {2.072_m, 6.513_m}; //Poner
  constexpr static const frc::Translation2d CenterPass = {2.486_m, 4.029_m}; //Poner
  constexpr static const frc::Translation2d RightPass = {2.098_m, 1.156_m}; //Poner

  static const InterpolatingTable<units::meter_t, units::degree_t> DistanceToHoodForHub{
      {
        {1.50_m, 0.0_deg},
        {2.00_m, 0.0_deg},
        {2.50_m, 0.25_deg},
        {3.00_m, 1.80_deg},
        {3.50_m, 3.25_deg},
        {4.00_m, 5.50_deg},
        {4.50_m, 6.30_deg},
        {5.00_m, 9.15_deg}
      }
  }; 


  static const InterpolatingTable<units::meter_t, units::turns_per_second_t> DistanceToShooterForHub{
    {
        {1.50_m, 27.25_tps},
        {2.00_m, 29.00_tps},
        {2.50_m, 29.80_tps},
        {3.00_m, 30.75_tps},
        {3.50_m, 31.75_tps},
        {4.00_m, 32.70_tps},
        {4.50_m, 33.70_tps},
        {5.00_m, 34.70_tps}
    }
  };

  static const InterpolatingTable<units::meter_t, units::degree_t> DistanceToHoodForLowPass{
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


  static const InterpolatingTable<units::meter_t, units::turns_per_second_t> DistanceToShooterForLowPass{
    {
        {0.1_m, 0_tps},
        {0.5_m, 0_tps},
        {1.0_m, 0_tps},
        {1.5_m, 0_tps},
        {2.0_m, 0_tps},
        {2.5_m, 0_tps},
        {3.0_m, 0_tps},
        {3.5_m, 0_tps},
        {4.0_m, 0_tps},
        {4.5_m, 0_tps},
    }
  };

  static const InterpolatingTable<units::meter_t, units::degree_t> DistanceToHoodForHighPass{
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


  static const InterpolatingTable<units::meter_t, units::turns_per_second_t> DistanceToShooterForHighPass{
    {
        {0.1_m, 0_tps},
        {0.5_m, 0_tps},
        {1.0_m, 0_tps},
        {1.5_m, 0_tps},
        {2.0_m, 0_tps},
        {2.5_m, 0_tps},
        {3.0_m, 0_tps},
        {3.5_m, 0_tps},
        {4.0_m, 0_tps},
        {4.5_m, 0_tps},
    }
  };

};
