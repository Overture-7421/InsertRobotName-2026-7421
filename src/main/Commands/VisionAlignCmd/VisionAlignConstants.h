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
		  {1.45_m, 5.00_deg},
		  {1.95_m, 8.50_deg},
		  {2.45_m, 11.50_deg},
		  {2.95_m, 15.50_deg},
		  {3.45_m, 18.00_deg},
		  {3.95_m, 21.00_deg},
		  {4.45_m, 25.00_deg}
		}
	};


	static const InterpolatingTable<units::meter_t, units::turns_per_second_t> DistanceToShooterForHub{
	  {
		  {1.45_m, 32.00_tps},
		  {1.95_m, 33.00_tps},
		  {2.45_m, 34.00_tps},
		  {2.95_m, 35.00_tps},
		  {3.45_m, 36.00_tps},
		  {3.95_m, 36.75_tps},
		  {4.45_m, 38.00_tps},
	  }
	};

	static const InterpolatingTable<units::meter_t, units::degree_t> DistanceToHoodForPass{
		{
		  {3.75_m, 15.0_deg},
		  {4.75_m, 18.0_deg},
		  {5.75_m, 21.0_deg},
		  {6.75_m, 24.0_deg},
		  {7.75_m, 27.0_deg},
		  {8.75_m, 28.5_deg},
		  {9.75_m, 28.5_deg}
		}
	};


	static const InterpolatingTable<units::meter_t, units::turns_per_second_t> DistanceToShooterForPass{
	  {
		  {3.75_m, 35.0_tps},
		  {4.75_m, 37.0_tps},
		  {5.75_m, 39.0_tps},
		  {6.75_m, 41.0_tps},
		  {7.75_m, 41.5_tps},
		  {8.75_m, 41.5_tps},
		  {9.75_m, 41.5_tps}
		}
	};

};
