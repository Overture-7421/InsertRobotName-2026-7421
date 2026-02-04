# include "Climber.h"


Climber::Climber(){};


void Climber::moveClimber(units::length::meter_t target_in_meters){
    units::degree_t target_in_degrees = units::degree_t(target_in_meters.value()/(ClimberConstants::Diameter.value()*M_PI));
    rightClimberMotor.SetControl(climberVoltage.WithPosition(target_in_degrees).WithEnableFOC(true));
};

bool Climber::isFinished(units::length::meter_t target_in_meters){
    units::degree_t target_in_degrees = units::degree_t(target_in_meters.value()/(ClimberConstants::Diameter.value()*M_PI));
    return abs(rightClimberMotor.GetPosition().GetValueAsDouble() - target_in_degrees.value())<1;

}

units::meter_t Climber::getPosition(){
    units::length::meter_t currentPosition = units::length::meter_t(rightClimberMotor.GetPosition().GetValueAsDouble()*(ClimberConstants::Diameter.value()*M_PI));
    return currentPosition;
}

frc2::CommandPtr Climber::SetPosition(units::length::meter_t target){
     return frc2::FunctionalCommand(
        // Init
        [this, target] { moveClimber(target); },
        // onExecute
        [this] { },
        // onEnd
        [this] (bool interrupted) { },
        // isFinished
        [this, target] { return isFinished(target);; }
    ).ToPtr();

}