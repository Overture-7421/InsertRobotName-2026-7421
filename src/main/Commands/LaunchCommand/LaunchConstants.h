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

	constexpr static const frc::Translation2d HubPose = { 4.6115986_m, 4.0213534_m }; //Poner 21 y 26
	constexpr static const frc::Translation2d LeftPass = { 2.000_m, 6.400_m }; //Poner
	constexpr static const frc::Translation2d RightPass = { 2.000_m, 1.600_m }; //Poner

	static const InterpolatingTable<units::meter_t, units::degree_t> DistanceToHoodForHub{
		{
		  {1.50_m, 0.0_deg},
		  {2.00_m, 0.0_deg},
		  {2.50_m, 1.0_deg},
		  {3.00_m, 2.0_deg},
		  {3.50_m, 5.5_deg},
		  {4.00_m, 7.75_deg},
		  {4.50_m, 9.5_deg},
		  {5.00_m, 10.0_deg}
		}
	};


	static const InterpolatingTable<units::meter_t, units::turns_per_second_t> DistanceToShooterForHub{
	  {
		  {1.50_m, 27.0_tps},
		  {2.00_m, 28.0_tps},
		  {2.50_m, 29.25_tps},
		  {3.00_m, 30.25_tps},
		  {3.50_m, 31.75_tps},
		  {4.00_m, 32.75_tps},
		  {4.50_m, 33.75_tps},
		  {5.00_m, 34.75_tps}
	  }
	};

	static const InterpolatingTable<units::meter_t, units::degree_t> DistanceToHoodForPass{
		{
		  {3.80_m, 10.0_deg},
		  {4.80_m, 11.0_deg},
		  {5.80_m, 12.0_deg},
		  {6.80_m, 15.5_deg},
		  {7.80_m, 17.0_deg},
		  {8.80_m, 20.0_deg},
		  {9.80_m, 24.0_deg},
		  {10.80_m, 27.0_deg},
		  {11.80_m, 28.0_deg},
		  {12.80_m, 29.0_deg},
		  {13.80_m, 30.0_deg}
		}
	};


	static const InterpolatingTable<units::meter_t, units::turns_per_second_t> DistanceToShooterForPass{
	  {
		  {3.80_m, 31.0_tps},
		  {4.80_m, 32.0_tps},
		  {5.80_m, 33.0_tps},
		  {6.80_m, 34.5_tps},
		  {7.80_m, 37.0_tps},
		  {8.80_m, 41.0_tps},
		  {9.80_m, 45.0_tps},
		  {10.80_m, 49.0_tps},
		  {11.80_m, 53.0_tps},
		  {12.80_m, 57.0_tps},
		  {13.80_m, 61.0_tps}
		}
	};

};
