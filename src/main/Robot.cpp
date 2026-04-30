// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc2/command/CommandScheduler.h>
#include <networktables/NetworkTableInstance.h>

void Robot::RobotInit() {

#ifndef __FRC_ROBORIO__
	simMotorManager.Init({
	  {8, "Shelby2/motors/back_left_drive"},
	  {7, "Shelby2/motors/back_left_rotation"},
	  {4, "Shelby2/motors/back_right_drive"},
	  {3, "Shelby2/motors/back_right_rotation"},
	  {6, "Shelby2/motors/front_left_drive"},
	  {5, "Shelby2/motors/front_left_rotation"},
	  {2, "Shelby2/motors/front_right_drive"},
	  {1, "Shelby2/motors/front_right_rotation"},

	  {23, "Shelby2/motors/hood"},
	  {19, "Shelby2/motors/indexer"},
	  {15, "Shelby2/motors/intake"},
	  {18, "Shelby2/motors/intake_roller"},
	  {21, "Shelby2/motors/passer"},
	  {25, "Shelby2/motors/shooter"}


		});

	simPigeonManager.Init("Shelby2/imu");

	simCANCoderManager.Init({

	  {12, "Shelby2/cancoders/back_right_cancoder"},
	  {11, "Shelby2/cancoders/back_left_cancoder"},
	  {10, "Shelby2/cancoders/front_left_cancoder"},
	  {9, "Shelby2/cancoders/front_right_cancoder"},

	  {24, "Shelby2/cancoders/hood_cancoder"},
	  {16, "Shelby2/cancoders/intake_cancoder"}

		});

	simDutyCycleEncoderManager.Init({});

	frc::AprilTagFieldLayout tagLayout = frc::AprilTagFieldLayout::LoadField(
		frc::AprilTagField::k2026RebuiltAndyMark);
	simPhotonVisionManager.Init(tagLayout);
#endif
	AddPeriodic([&] {
		frc2::CommandScheduler::GetInstance().Run();
#ifndef __FRC_ROBORIO__
		nt::NetworkTableInstance::GetDefault().Flush();
#endif
	}, RobotConstants::LoopTime, RobotConstants::TimingOffset);

}

/**
 * This function is called every 20 ms, no matter the mode. Use
 *
 * this for items like diagnostics that you want to run during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
	frc::SmartDashboard::PutNumber("Pigeon Yaw", m_container.chassis.getRotation2d().Degrees().value());
	m_container.UpdateTelemetry();
}

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() {

}

void Robot::DisabledPeriodic() {}

/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */
void Robot::AutonomousInit() {
	m_autonomousCommand = m_container.GetAutonomousCommand();

	if (m_autonomousCommand) {
		frc2::CommandScheduler::GetInstance().Schedule(m_autonomousCommand);
	}


}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {

	frc2::CommandScheduler::GetInstance().CancelAll();
	frc2::CommandScheduler::GetInstance().Schedule(m_container.processor.setProcessorCmd(ProcessorConstants::Stop));
	frc2::CommandScheduler::GetInstance().Schedule(m_container.intake.setRollersCmd(IntakeConstants::IntakeSustain.rollers));
	frc2::CommandScheduler::GetInstance().Schedule(m_container.hood.setHoodAngleCommand(HoodConstants::Close));

	m_container.chassis.setXMode(false);

}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() {}

/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {}

/**
 * This function is called once when the robot is first started up.
 */
void Robot::SimulationInit() {}

/**
 * This function is called periodically whilst in simulation.
 */
void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
	return frc::StartRobot<Robot>();
}
#endif