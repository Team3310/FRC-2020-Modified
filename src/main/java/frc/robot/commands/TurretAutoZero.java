/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Turret;

public class TurretAutoZero extends ExtraTimeoutCommand
{
    private final Turret turret;
    private boolean isHomeFound;

    public TurretAutoZero(Turret subsystem)
    {
        this.turret = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        turret.setTurretSpeed(Constants.TURRET_AUTO_ZERO_SPEED);
        isHomeFound = false;
        resetExtraOneTimer();
        startExtraOneTimeout(5.0);
    }

    @Override
    public boolean isFinished() {
        if (turret.getMinTurretSensor() == true) {
            isHomeFound = true;
            return true;
        }
        if (isExtraOneTimedOut()) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        turret.setTurretSpeed(0.0);
        if (isHomeFound) {
            turret.resetHomePosition(Constants.TURRET_AUTO_HOME_POSITION_DEGREES);
//            turret.setTurretPositionRelative(0);
            System.out.println("HOME RESET");
        }
        else {
            System.out.println("HOME NOT FOUND");
        }
    }
}
