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

	constexpr static const frc::Translation2d HubPose = { 4.6115986_m, 4.0213534_m }; //Poner
	constexpr static const frc::Translation2d LeftPass = { 2.000_m, 6.400_m }; //Poner
	constexpr static const frc::Translation2d RightPass = { 2.000_m, 1.600_m }; //Poner

	static const InterpolatingTable<units::meter_t, units::degree_t> DistanceToHoodForHub{
		{
		  {1.50_m, 0.0_deg},
		  {2.00_m, 0.0_deg},
		  {2.50_m, 0.25_deg},
		  {3.00_m, 1.80_deg},
		  {3.50_m, 3.25_deg},
		  {4.00_m, 5.50_deg},
		  {4.50_m, 7.5_deg},
		  {5.00_m, 8.5_deg}
		}
	};


	static const InterpolatingTable<units::meter_t, units::turns_per_second_t> DistanceToShooterForHub{
	  {
		  {1.50_m, 26.0_tps},
		  {2.00_m, 26.25_tps},
		  {2.50_m, 28.0_tps},
		  {3.00_m, 29.5_tps},
		  {3.50_m, 31.0_tps},
		  {4.00_m, 31.3_tps},
		  {4.50_m, 33.5_tps},
		  {5.00_m, 34_tps}
	  }
	};

	static const InterpolatingTable<units::meter_t, units::degree_t> DistanceToHoodForPass{
		{
		  {3.5_m, 12.0_deg},
		  {4.0_m, 12.0_deg},
		  {4.5_m, 12.0_deg},
		  {5.0_m, 12.0_deg},
		  {5.5_m, 12.0_deg},
		  {6.0_m, 12.0_deg},
		  {6.5_m, 12.0_deg},
		  {7.0_m, 12.0_deg},
		  {7.5_m, 12.0_deg},
		  {8.0_m, 12.0_deg},
		  {8.5_m, 12.0_deg},
		  {9.0_m, 12.0_deg}
		}
	};


	static const InterpolatingTable<units::meter_t, units::turns_per_second_t> DistanceToShooterForPass{
	  {
		  {3.5_m, 30.0_tps},
		  {4.0_m, 30.5_tps},
		  {4.5_m, 31.0_tps},
		  {5.0_m, 31.5_tps},
		  {5.5_m, 32.0_tps},
		  {6.0_m, 32.5_tps},
		  {6.5_m, 33.0_tps},
		  {7.0_m, 33.5_tps},
		  {7.5_m, 34.0_tps},
		  {8.0_m, 34.5_tps},
		  {8.5_m, 35.0_tps},
		  {9.0_m, 35.5_tps}
		}
	};

};
