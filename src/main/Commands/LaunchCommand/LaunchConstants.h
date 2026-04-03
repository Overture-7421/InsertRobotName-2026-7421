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
		  {1.45_m, 3.00_deg},
		  {1.95_m, 6.00_deg},
		  {2.45_m, 8.50_deg},
		  {2.95_m, 12.00_deg},
		  {3.45_m, 15.50_deg},
		  {3.95_m, 17.25_deg},
		  {4.45_m, 9.5_deg}, //Falta
		  {4.95_m, 10.0_deg} //Falta
		}
	};


	static const InterpolatingTable<units::meter_t, units::turns_per_second_t> DistanceToShooterForHub{
	  {
		  {1.45_m, 31.00_tps},
		  {1.95_m, 32.50_tps},
		  {2.45_m, 33.50_tps},
		  {2.95_m, 34.25_tps},
		  {3.45_m, 35.00_tps},
		  {3.95_m, 36.00_tps},
		  {4.45_m, 33.75_tps}, //Falta
		  {4.95_m, 34.75_tps}  //Falta
	  }
	};

	static const InterpolatingTable<units::meter_t, units::degree_t> DistanceToHoodForPass{
		{
		  {3.62_m, 16.0_deg},
		  {4.62_m, 18.0_deg},
		  {5.62_m, 21.0_deg},
		  {6.62_m, 23.5_deg},
		  {7.62_m, 24.0_deg}
		}
	};


	static const InterpolatingTable<units::meter_t, units::turns_per_second_t> DistanceToShooterForPass{
	  {
		  {3.62_m, 34.0_tps},
		  {4.62_m, 36.0_tps},
		  {5.62_m, 37.0_tps},
		  {6.62_m, 39.0_tps},
		  {7.62_m, 41.0_tps}
		}
	};

};
