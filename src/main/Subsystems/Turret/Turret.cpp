// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Turret.h"

Turret::Turret(Chassis* chassis) {

    turretPID.DisableContinuousInput();
    turretPID.SetTolerance(TurretConstants::TurretRangeOfError);
    
    units::degree_t target = calculateTurretAngleFromCANCoderDegrees();
    frc::SmartDashboard::PutNumber("TurretData/Start Position", target.value());
    turretMotor.SetPosition(target);

    this->chassis = chassis;


}

void Turret::setTargetAngle(units::degree_t turretTarget) {
    frc::SmartDashboard::PutNumber("TurretData/Target Position", turretTarget.value());
    this->target = turretTarget;
}

units::degree_t Turret::convertToClosestBoundedTurretAngleDegrees(units::degree_t targetAngleDegrees){
    units::degree_t currentTotalDegrees = turretMotor.GetPosition().GetValue() * 360.0;

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

    frc::SmartDashboard::PutNumber("TurretData/Encoder1", encoder1.value() / 360.0);
    frc::SmartDashboard::PutNumber("TurretData/Encoder2", encoder2.value() / 360.0);


   units::degree_t difference = encoder2 - encoder1;
   if(difference > 250.0_deg){ // Maybe estos numeros se modifican al ver los rangos de la torreta
    difference -= 360.0_deg;
   }  
   if (difference < -250.0_deg){
    difference += 360.0_deg;
   }
   
   units::degree_t estimatedTurretAngle = difference * TurretConstants::Slope;

   double encoder1Rotations = (estimatedTurretAngle * TurretConstants::GearRatioTurretToEncoder1) / 360.0_deg;
   double encoder1RotationsFloored = std::floor(encoder1Rotations);

   units::degree_t turretAngle = (encoder1RotationsFloored * 360.0_deg + encoder1) * TurretConstants::GearRatioEncoder1ToTurret;

   units::degree_t degreesPerEncoder1Rotation = 360.0_deg * TurretConstants::GearRatioEncoder1ToTurret;
   units::degree_t error = turretAngle - estimatedTurretAngle;

   if(error < -100.0_deg){
    turretAngle += degreesPerEncoder1Rotation;
   } else if (error > 100.0_deg){
    turretAngle -= degreesPerEncoder1Rotation;
   }

    frc::SmartDashboard::PutNumber("TurretData/EncodersCombined", turretAngle.value());
   return turretAngle; 

}

frc::Rotation2d Turret::GetTurretAimingParameterFromRobotPose(const frc::Pose2d& robotPose, const frc::Translation2d& targetPosition){
    frc::Transform2d robotToTurret {
        frc::Translation2d{0.0_in, 0.0_in}, //Posicion del robot al centro de la torreta
        frc::Rotation2d{0.0_deg} //Orientacion de la torreta respecto al robot
    };

    frc::Pose2d turretPose = robotPose.TransformBy(robotToTurret); //Posicion Global de la Torreta

    units::meter_t deltaX = targetPosition.X() - turretPose.X(); //Para calcular el Angulo Absoluto al Target
    units::meter_t deltaY = targetPosition.Y() - turretPose.Y();
    units::radian_t targetAbsAngle = units::math::atan2(deltaY, deltaX); //Angulo Absoluto de la Cancha

    units::radian_t angleDifference = targetAbsAngle - robotPose.Rotation().Radians(); //Cuanto debe girar la Torreta respecto al Chassis
    units::radian_t constrainedAngle = frc::AngleModulus(angleDifference); //Convierte el resultado a un rango de -180 a 180°

    return frc::Rotation2d {constrainedAngle}; //Retorna el Angulo Relativo que debe girar la Torreta
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


bool Turret::isAimAtFieldPosition(units::degree_t setPoint){
    units::degree_t currentAngle = calculateTurretAngleFromCANCoderDegrees();
    units::degree_t error = units::math::abs(setPoint - currentAngle);
    return error < 2.0_deg;
}

void Turret::Periodic() {
    
    // units::volt_t motorOutput = units::volt_t(turretPID.Calculate(calculateTurretAngleFromCANCoderDegrees(), {target, -chassis->getCurrentSpeeds().omega}));
    units::volt_t motorOutput = units::volt_t(turretPID.Calculate(calculateTurretAngleFromCANCoderDegrees(), {target, 0.0_rad_per_s}));
    turretMotor.SetControl(turretVoltageRequest.WithOutput(motorOutput).WithEnableFOC(true));

    units::degree_t realAngle = calculateTurretAngleFromCANCoderDegrees();
    units::degree_t motorAngle = turretMotor.GetPosition().GetValue();
    units::degree_t error = units::math::abs(realAngle - motorAngle);
    if (error > 2.0_deg){
        turretMotor.SetPosition(realAngle);
    }
    frc::SmartDashboard::PutNumber("TurretData/Real Angle", realAngle.value() / 360.0);
    frc::SmartDashboard::PutNumber("TurretData/Motor Angle", motorAngle.value());
    frc::SmartDashboard::PutNumber("TurretData/Error", error.value());

}
  