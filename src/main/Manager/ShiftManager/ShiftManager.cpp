// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Manager/ShiftManager/ShiftManager.h"
#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Timer.h>

ShiftManager::ShiftManager() {
}

HubState ShiftManager::GetHubState() const {
  HubState state;
  std::optional<frc::DriverStation::Alliance> alliance = frc::DriverStation::GetAlliance();

  if (!alliance.has_value() || !frc::DriverStation::IsTeleopEnabled()) {
    return state; // False, false
  }

  if (frc::DriverStation::IsAutonomousEnabled()) {
    state.isActive = true;
    return state;
  }

  double matchTime = frc::DriverStation::GetMatchTime().value();
  std::string gameData = frc::DriverStation::GetGameSpecificMessage();

  if (gameData.empty()) {
    state.isActive = true;
    return state;
  }

  bool redInactiveFirst = false;
  switch (gameData[0]) {
    case 'R': redInactiveFirst = true; break;
    case 'B': redInactiveFirst = false; break;
    default: 
      state.isActive = true; 
      return state;
  }

  bool shift1Active = (alliance.value() == frc::DriverStation::Alliance::kRed) ? !redInactiveFirst : redInactiveFirst;
  const double warningTime = 3.0; 

  if (matchTime > 130) {
    state.isActive = true;
    state.isTransitioning = (matchTime - 130.0) <= warningTime;
  } else if (matchTime > 105) {
    state.isActive = shift1Active;
    state.isTransitioning = (matchTime - 105.0) <= warningTime;
  } else if (matchTime > 80) {
    state.isActive = !shift1Active;
    state.isTransitioning = (matchTime - 80.0) <= warningTime;
  } else if (matchTime > 55) {
    state.isActive = shift1Active;
    state.isTransitioning = (matchTime - 55.0) <= warningTime;
  } else if (matchTime > 30) {
    state.isActive = !shift1Active;
    state.isTransitioning = (matchTime - 30.0) <= warningTime;
  } else {
    state.isActive = true;
    state.isTransitioning = false; 
  }

  return state;
}

void ShiftManager::Periodic() {
  // Get the current match time and state
  double matchTime = frc::DriverStation::GetMatchTime().value();
  HubState currentState = GetHubState();

  // Post the raw time and booleans for debugging
  frc::SmartDashboard::PutNumber("Hub/MatchTime", matchTime);
  frc::SmartDashboard::PutBoolean("Hub/IsActive", currentState.isActive);
  frc::SmartDashboard::PutBoolean("Hub/IsTransitioning", currentState.isTransitioning);

  // Post a human-readable string for the drivers
  std::string driverStatus;
    double currentTime = frc::Timer::GetFPGATimestamp().value();
  bool blinkOn = std::fmod(currentTime, 0.4) < 0.2;

  // 2. These booleans will drive the colored boxes on the dashboard
  bool showGreenBox = false;
  bool showRedBox = false;

  if (currentState.isActive && !currentState.isTransitioning) {
    driverStatus = "ACTIVE";
    showGreenBox = true; // Solid Green
    
  } else if (currentState.isActive && currentState.isTransitioning) {
    // If blinkOn is true, show text. If false, show nothing.
    driverStatus = blinkOn ? "ENDING SOON" : ""; 
    showGreenBox = blinkOn; // Blinking Green
    
  } else if (!currentState.isActive && !currentState.isTransitioning) {
    driverStatus = "INACTIVE";
    showRedBox = true; // Solid Red
    
  } else if (!currentState.isActive && currentState.isTransitioning) {
    driverStatus = blinkOn ? "ALMOST ACTIVE" : ""; 
    showRedBox = blinkOn; // Blinking Red
  }

  // 3. Send the blinking text to the dashboard
  frc::SmartDashboard::PutString("Hub/DriverStatus", driverStatus);

  // 4. Send the booleans to drive the colors
  frc::SmartDashboard::PutBoolean("Hub/GreenIndicator", showGreenBox);
  frc::SmartDashboard::PutBoolean("Hub/RedIndicator", showRedBox);
} 