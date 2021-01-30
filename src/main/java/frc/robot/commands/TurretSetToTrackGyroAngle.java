/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class TurretSetToTrackGyroAngle extends CommandBase
{
    private final Turret turret;
    private double offsetAngleDeg;

    public TurretSetToTrackGyroAngle(Turret subsystem, double offsetAngleDeg)
    {
        this.turret = subsystem;
        this.offsetAngleDeg = offsetAngleDeg;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        turret.setGyroTrackMode(offsetAngleDeg);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
