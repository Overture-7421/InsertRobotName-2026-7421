// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Turret.h"

Turret::Turret(Chassis* chassis) {

    // turretPID.SetTolerance(TurretConstants::TurretRangeOfError);
    turretMotor.setSensorToMechanism(TurretConstants::SensorToMechanism);
    turretMotor.SetPosition(calculateTurretAngleFromCANCoderDegrees());

    this->chassis = chassis;

}

void Turret::setTargetAngle(units::degree_t turretTarget) {
    frc::SmartDashboard::PutNumber("TurretData/Target Position", turretTarget.value());
    turretMotor.SetControl(turretVoltageRequest.WithPosition(turretTarget).WithFeedForward(units::volt_t(-chassis->getCurrentSpeeds().omega.value() * 0.0)));

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

    frc::SmartDashboard::PutNumber("TurretData/Final Target Position", finalOffset.value());
    return finalOffset;
}

units::degree_t Turret::calculateTurretAngleFromCANCoderDegrees(){
    units::degree_t encoder1 = turret1CANCoder.GetAbsolutePosition().GetValue();
    units::degree_t encoder2 = turret2CANCoder.GetAbsolutePosition().GetValue();

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

   return turretAngle; 

}

frc::Rotation2d Turret::GetTurretAimingParameterFromRobotPose(const frc::Pose2d& robotPose, const frc::Translation2d& targetPosition){
    frc::Transform2d robotToTurret {
        frc::Translation2d{-5.2_in, 4.2_in}, //Posicion del robot al centro de la torreta
        frc::Rotation2d{0.0_deg} //Orientacion de la torreta respecto al robot
    };

    frc::Pose2d turretPose = robotPose.TransformBy(robotToTurret); //Posicion Global de la Torreta

    units::meter_t deltaX = targetPosition.X() - turretPose.X(); //Para calcular el Angulo Absoluto al Target
    units::meter_t deltaY = targetPosition.Y() - turretPose.Y();
    units::degree_t targetAbsAngle = units::math::atan2(deltaY, deltaX); //Angulo Absoluto de la Cancha
    frc::SmartDashboard::PutNumber("TurretData/TargetAbsAngle", targetAbsAngle.value());

    units::degree_t angleDifference = targetAbsAngle - robotPose.Rotation().Degrees(); //Cuanto debe girar la Torreta respecto al Chassis
    units::degree_t constrainedAngle = frc::AngleModulus(angleDifference); //Convierte el resultado a un rango de -180 a 180°
    frc::SmartDashboard::PutNumber("TurretData/TurretAngleRelativeToRobot", constrainedAngle.value());
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


bool Turret::isAimAtFieldPosition(const frc::Pose2d& robotPose, const frc::Translation2d& targetPosition){
    frc::Rotation2d idealAngle = GetTurretAimingParameterFromRobotPose(robotPose, targetPosition);
    units::degree_t setPoint = convertToClosestBoundedTurretAngleDegrees(idealAngle.Degrees());

    units::degree_t currentAngle = calculateTurretAngleFromCANCoderDegrees();
    units::degree_t error = units::math::abs(setPoint - currentAngle);
    return error < 2.0_deg;
}

void Turret::UpdateTelemetry(){
    units::degree_t encoder1 = turret1CANCoder.GetAbsolutePosition().GetValue();
    units::degree_t encoder2 = turret2CANCoder.GetAbsolutePosition().GetValue();
    frc::SmartDashboard::PutNumber("TurretData/Encoder1", encoder1.value());
    frc::SmartDashboard::PutNumber("TurretData/Encoder2", encoder2.value());

    frc::SmartDashboard::PutNumber("TurretData/EncodersCombined", calculateTurretAngleFromCANCoderDegrees().value());


}

void Turret::Periodic() {
    
    // units::volt_t motorOutput = units::volt_t(turretPID.Calculate(calculateTurretAngleFromCANCoderDegrees(), target) - chassis->getCurrentSpeeds().omega.value() * 1.0);
    // turretMotor.SetControl(turretVoltageRequest.WithOutput(motorOutput).WithEnableFOC(true));    

}
  