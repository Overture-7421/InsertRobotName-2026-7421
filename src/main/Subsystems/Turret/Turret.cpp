// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Turret.h"

Turret::Turret(Chassis* chassis) {

    // turretPID.SetTolerance(TurretConstants::TurretRangeOfError);
    turretMotor.setSensorToMechanism(TurretConstants::SensorToMechanism);
    turretMotor.SetPosition(calculateTurretAngleFromCANCoderDegrees());
    turretMotor.configureMotionMagic(TurretConstants::TurretVelocity, TurretConstants::TurretAcceleration, 0.0_tr_per_s_cu);

    // frc::SmartDashboard::PutNumber("TurretPose/TurretPoseX", 0.0);
    // frc::SmartDashboard::PutNumber("TurretPose/TurretPoseY", 0.0);

    this->chassis = chassis;

}

void Turret::setTargetAngle(units::degree_t turretTarget) {
    turretMotor.SetControl(turretVoltageRequest.WithPosition(turretTarget).WithFeedForward(units::volt_t(chassis->getCurrentSpeeds().omega.value() * TurretConstants::ChassisAngularVelocityCompensator)));
    //.WithFeedForward(units::volt_t(getForceFactorCables(calculateTurretAngleFromCANCoderDegrees()) * TurretConstants::CableSpringConstant)) Por si es necesario

}

units::degree_t Turret::convertToClosestBoundedTurretAngleDegrees(units::degree_t targetAngleDegrees){
    units::degree_t currentTotalDegrees = turretMotor.GetPosition().GetValue();

    units::degree_t closestOffset = targetAngleDegrees - units::math::fmod(currentTotalDegrees, 360.0_deg);
    if(closestOffset > 180.0_deg){
        closestOffset -= 360.0_deg;
    } else if (closestOffset < -180.0_deg){
        closestOffset += 360.0_deg;
    }

    units::degree_t finalOffset = currentTotalDegrees + closestOffset;
    if(units::math::fmod(currentTotalDegrees + closestOffset, 360.0_deg) == units::math::fmod(currentTotalDegrees - closestOffset, 360.0_deg)){
        if(finalOffset > 0.0_deg){ // Creo que seria la mitad primera del circulo entero de la torreta
            finalOffset = currentTotalDegrees - units::math::abs(closestOffset);
        } else {
            finalOffset = currentTotalDegrees + units::math::abs(closestOffset);
        }
    }
    if(finalOffset > TurretConstants::TurretForwardLimit){
        finalOffset -= 360.0_deg;
    } else if (finalOffset < TurretConstants::TurretReverseLimit){
        finalOffset += 360.0_deg;
    }

    return finalOffset;
}

units::degree_t Turret::calculateTurretAngleFromCANCoderDegrees(){
    units::degree_t encoder1 = turret1CANCoder.GetAbsolutePosition().GetValue();
    units::degree_t encoder2 = turret2CANCoder.GetAbsolutePosition().GetValue();

   units::degree_t difference = encoder2 - encoder1;
   if(difference > 180.0_deg){
        difference -= 360.0_deg;
    }  
    if(difference < -180.0_deg){
        difference += 360.0_deg;
    }
   
   units::degree_t estimatedTurretAngle = difference * TurretConstants::Slope;

   double encoder1Rotations = (estimatedTurretAngle * TurretConstants::GearRatioTurretToEncoder1) / 360.0_deg;
   double encoder1RotationsFloored = std::floor(encoder1Rotations);

   units::degree_t turretAngle = (encoder1RotationsFloored * 360.0_deg + encoder1) * TurretConstants::GearRatioEncoder1ToTurret;

   units::degree_t degreesPerEncoder1Rotation = 360.0_deg * TurretConstants::GearRatioEncoder1ToTurret;
   units::degree_t error = turretAngle - estimatedTurretAngle;

   if(error < -50.0_deg){               // Changed from -100
    turretAngle += degreesPerEncoder1Rotation;
   } else if (error > 50.0_deg){        // Changed from 100
    turretAngle -= degreesPerEncoder1Rotation;
   }

   return turretAngle; 
   

}

double Turret::getForceFactorCables(units::degree_t turretAngleDegrees) {
    if(turretAngleDegrees > TurretConstants::TurretSafetyZoneCablesForward){
        // return (units::math::abs(turretAngleDegrees) / turretAngleDegrees) * ((units::math::abs(turretAngleDegrees) - TurretConstants::TurretSafetyZoneCables) / TurretConstants::TurretForceAppliedRange);
        return 1.0;
    } else if (turretAngleDegrees < -TurretConstants::TurretSafetyZoneCablesReverse) {
        return -1.0;
    } else {
        return 0.0;
    }
}

frc::Rotation2d Turret::GetTurretAimingParameterFromRobotPose(const frc::Pose2d& robotPose, const frc::Translation2d& targetPosition){
    frc::Pose2d turretPose = GetTurretPose(robotPose); //Posicion Global de la Torreta


    units::meter_t deltaX = targetPosition.X() - turretPose.X(); //Para calcular el Angulo Absoluto al Target
    units::meter_t deltaY = targetPosition.Y() - turretPose.Y();
    units::degree_t targetAbsAngle = units::math::atan2(deltaY, deltaX); //Angulo Absoluto de la Cancha

    units::degree_t angleDifference = targetAbsAngle - robotPose.Rotation().Degrees(); //Cuanto debe girar la Torreta respecto al Chassis
    units::degree_t constrainedAngle = frc::AngleModulus(angleDifference); //Convierte el resultado a un rango de -180 a 
    
    turretPublisher.Set({turretPose.X(), turretPose.Y(), {convertToClosestBoundedTurretAngleDegrees(constrainedAngle) + robotPose.Rotation().Degrees()}}); //Publica la Posicion de la Torreta a NetworkTables para su visualizacion en el Dashboard

    return frc::Rotation2d {constrainedAngle + 2.0_deg}; //Retorna el Angulo Relativo que debe girar la Torreta
}

frc::Pose2d Turret::GetTurretPose(const frc::Pose2d& robotPose){
    // units::inch_t turretPoseX = units::inch_t(frc::SmartDashboard::GetNumber("TurretPose/TurretPoseX",0.0));
    // units::inch_t turretPoseY = units::inch_t(frc::SmartDashboard::GetNumber("TurretPose/TurretPoseY",0.0));

    return robotPose.TransformBy(frc::Transform2d(GetRobotToTurret().X(), GetRobotToTurret().Y(), GetRobotToTurret().Rotation().ToRotation2d())); //Posicion Global de la Torreta
}

const frc::Transform3d& Turret::GetRobotToTurret(){
	return robotToTurret;
}

void Turret::AimAtFieldPosition(const frc::Pose2d& robotPose, const frc::Translation2d& targetPosition){
    frc::Rotation2d idealAngle = GetTurretAimingParameterFromRobotPose(robotPose, targetPosition);
    units::degree_t setPoint = convertToClosestBoundedTurretAngleDegrees(idealAngle.Degrees());
    setTargetAngle(setPoint);
}

frc2::CommandPtr Turret::TestCommand(units::degree_t setPoint){
    return frc2::cmd::RunOnce([this, setPoint]{
        this->setTargetAngle(setPoint);
    });
}


bool Turret::isAimAtFieldPosition(const frc::Pose2d& robotPose, const frc::Translation2d& targetPosition){
    frc::Rotation2d idealAngle = GetTurretAimingParameterFromRobotPose(robotPose, targetPosition);
    units::degree_t setPoint = convertToClosestBoundedTurretAngleDegrees(idealAngle.Degrees());

    units::degree_t currentAngle = GetRobotRelativeHeading();
    units::degree_t error = units::math::abs(setPoint - currentAngle);
    return error < 3.0_deg;
}

bool Turret::isMotorAtPosition(){
    return turretMotor.GetClosedLoopError().GetValue() < 2.0;
}

void Turret::UpdateTelemetry(){
    units::degree_t encoder1 = turret1CANCoder.GetAbsolutePosition().GetValue();
    units::degree_t encoder2 = turret2CANCoder.GetAbsolutePosition().GetValue();
    frc::SmartDashboard::PutNumber("TurretData/Encoder1", encoder1.value());
    frc::SmartDashboard::PutNumber("TurretData/Encoder2", encoder2.value());

    frc::SmartDashboard::PutNumber("TurretData/EncodersCombined", GetRobotRelativeHeading().value());
    frc::SmartDashboard::PutNumber("TurretData/MotorAngle", turretMotor.GetPosition().GetValue().value() * 360.0);

    double targetAngle = turretMotor.GetClosedLoopReference().GetValue();
    frc::SmartDashboard::PutNumber("TurretData/ErrorAngle", turretMotor.GetClosedLoopError().GetValue());
    frc::SmartDashboard::PutNumber("TurretData/TargetAngle", targetAngle);

}

const units::degree_t& Turret::GetRobotRelativeHeading(){
	return turretActualAngle;
}

void Turret::Periodic() {
    turretActualAngle = calculateTurretAngleFromCANCoderDegrees();
}
  