#pragma once
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Pose2d.h>

class PassTargetSwitcher {
public:
    PassTargetSwitcher(const frc::Translation2d& leftPassTarget, const frc::Translation2d& rightPassTarget, const units::meter_t& overlap) {
        this->leftPassTarget = leftPassTarget;
        this->rightPassTarget = rightPassTarget;
        this->overlap = overlap;
    }

    const frc::Translation2d& GetPassTarget(const frc::Pose2d& chassisPose, bool redAlliance) {
        switch (currentTargetSide)
        {
        case TargetSide::None:
            if (chassisPose.Y() > midpoint) {
                currentTargetSide = TargetSide::Left;
            }else{
                currentTargetSide = TargetSide::Right;
            }
            break;
        case TargetSide::Left:
            if(chassisPose.Y() < midpoint - overlap) {
                currentTargetSide = TargetSide::Right;
            }
            break;
        case TargetSide::Right:
            if(chassisPose.Y() > midpoint + overlap) {
                currentTargetSide = TargetSide::Left;
            }
            break;
        default:
            break;
        }

        if (redAlliance){
            return currentTargetSide == TargetSide::Left ? rightPassTarget : leftPassTarget;
        }else{
            return currentTargetSide == TargetSide::Left ? leftPassTarget : rightPassTarget;
        }
    }

private:
    enum class TargetSide {
        None,
        Left,
        Right
    };
    frc::Translation2d rightPassTarget;
    frc::Translation2d leftPassTarget;
    units::meter_t overlap;
    TargetSide currentTargetSide = TargetSide::None;
    const units::meter_t midpoint = 4_m;
};