/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class HoodSetToTrackLimelightAngle extends CommandBase
{
    private final Shooter shooter;

    public HoodSetToTrackLimelightAngle(Shooter subsystem)
    {
        this.shooter = subsystem;
    }

    @Override
    public void initialize() {
        shooter.setLimelightHoodTrackMode();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Hood interrupted = " + interrupted);
    }
}
