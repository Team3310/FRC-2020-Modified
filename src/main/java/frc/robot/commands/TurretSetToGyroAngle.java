/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Turret;
import frc.robot.utilities.Util;

public class TurretSetToGyroAngle extends ExtraTimeoutCommand
{
    private final Turret turret;
    private double offsetAngleDeg;

    public TurretSetToGyroAngle(Turret subsystem, double offsetAngleDeg)
    {
        this.turret = subsystem;
        this.offsetAngleDeg = offsetAngleDeg;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        double gyroMirror = Util.normalizeAngle90ToMinus270(Drive.getInstance().getGyroFusedHeadingAngleDeg());
        if (Math.abs(gyroMirror) < 90) {
            gyroMirror = -gyroMirror;
        }
        else {
            gyroMirror = (-180 - gyroMirror) - 180;
        }
        turret.setTurretMotionMagicPositionAbsolute(Util.normalizeAngle90ToMinus270(gyroMirror) + offsetAngleDeg);
        resetExtraOneTimer();
        startExtraOneTimeout(0.1);
    }

    @Override
    public boolean isFinished() {
        if (isExtraOneTimedOut() && turret.hasFinishedTrajectory()) {
            System.out.println("Gyro Turret index angle = finished");
            return true;
        }
        return false;
    }
}
