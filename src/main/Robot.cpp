// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc2/command/CommandScheduler.h>


// Gearings
// Motores correctos cantidad
// Agregar motores en init
// Cancoders correctos ()

// Elevador


Robot::Robot() {

  #ifndef __FRC_ROBORIO__ 
    simMotorManager.Init({
      {8, "Rebuilt2026/motors/back_right_drive"},
      {6, "Rebuilt2026/motors/back_left_drive"},
      {2, "Rebuilt2026/motors/front_left_drive"},
      {4, "Rebuilt2026/motors/front_right_drive"},
      
      {7, "Rebuilt2026/motors/back_right_rotation"},
      {5, "Rebuilt2026/motors/back_left_rotation"},
      {1, "Rebuilt2026/motors/front_left_rotation"},
      {3, "Rebuilt2026/motors/front_right_rotation"},

      {23,"Rebuilt2026/motors/turret"},
      {17,"Rebuilt2026/motors/spindexer"},
      {21,"Rebuilt2026/motors/shooterWheels"},
      {16,"Rebuilt2026/motors/intakeRollers"},
      {14,"Rebuilt2026/motors/intake"},
      {19,"Rebuilt2026/motors/hood"},
      {26,"Rebuilt2026/motors/elevatorRight"},
      {25,"Rebuilt2026/motors/elevatorLeft"}

  
        });

    simPigeonManager.Init("Rebuilt2026/imu");

    simCANCoderManager.Init({
      {11, "Rebuilt2026/cancoders/back_right_cancoder"},
      {10, "Rebuilt2026/cancoders/back_lmeft_cancoder"},
      {9, "Rebuilt2026/cancoders/front_left_cancoder"},
      {12, "Rebuilt2026/cancoders/front_right_cancoder"},

      {20, "Rebuilt2026/cancoders/hood"},
      {15, "Rebuilt2026/cancoders/intake"},
      {24, "Rebuilt2026/cancoders/turret"}
        });

    simDutyCycleEncoderManager.Init({});
#endif

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
  frc2::CommandScheduler::GetInstance().Run();

}

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() {}

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
  // This makes sure that the autonomous stops running when
  // teleop starts running. If you want the autonomous to
  // continue until interrupted by another command, remove
  // this line or comment it out.

}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() {
}

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