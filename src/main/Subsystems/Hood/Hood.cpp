// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Hood.h"

Hood::Hood(){
    hoodMotor.setRotorToSensorRatio(HoodConstants::RotorToSensor);
    hoodMotor.setSensorToMechanism(HoodConstants::SensorToMechanism);

    hoodMotor.setFusedCANCoder(hoodCANCoder.GetDeviceID());
    
    hoodMotor.configureMotionMagic(HoodConstants::CruiseVelocity,
                                  HoodConstants::CruiseAcceleration,
                                  0.0_tr_per_s_cu);
}

void Hood::setHoodAngle(units::degree_t angle){
    hoodMotor.SetControl(hoodVoltageRequest.WithPosition(angle).WithEnableFOC(false));

}

units::degree_t Hood::getHoodAngle(){
    return hoodMotor.GetPosition().GetValue();
}

bool Hood::isHoodAtAngle(units::degree_t targetAngle){
    units::degree_t hoodError = targetAngle - hoodMotor.GetPosition().GetValue();
    return units::math::abs(hoodError) < HoodConstants::RangeOfError;
}

frc2::CommandPtr Hood::setHoodAngleCommand(units::degree_t angle){
    return frc2::cmd::RunOnce(
        [this, angle] {
            setHoodAngle(angle);
        }, {this}
    );
}


void Hood::UpdateTelemetry(){
    frc::SmartDashboard::PutNumber("Hood/ActualAngle", hoodMotor.GetPosition().GetValue().convert<units::degree>().value());
    frc::SmartDashboard::PutNumber("Hood/EncoderAngle", hoodCANCoder.GetAbsolutePosition().GetValue().convert<units::degree>().value());

    double targetAngle = hoodMotor.GetClosedLoopReference().GetValue();
    frc::SmartDashboard::PutNumber("Hood/ErrorAngle", hoodMotor.GetClosedLoopError().GetValue());
    frc::SmartDashboard::PutNumber("Hood/TargetAngle", targetAngle * 360.0);
    frc::SmartDashboard::PutBoolean("Hood/isHoodAtAngle", isHoodAtAngle(units::turn_t(targetAngle)));
}

void Hood::Periodic() {}
