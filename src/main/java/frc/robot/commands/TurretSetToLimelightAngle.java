/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Turret;

public class TurretSetToLimelightAngle extends ExtraTimeoutCommand
{
    private final Turret turret;
    private final Limelight limelight;
    private double offsetAngleDeg;
    private boolean targetFound;

    public TurretSetToLimelightAngle(Turret subsystem, double offsetAngleDeg)
    {
        this.turret = subsystem;
        this.offsetAngleDeg = offsetAngleDeg;
        this.limelight = Limelight.getInstance();
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        limelight.setCameraMode(Limelight.CameraMode.VISION);
        limelight.setPipeline(0);
        if (limelight.isOnTarget()) {
            targetFound = true;
            System.out.println("Limelight tracking = " + -limelight.getTx());
            turret.setTurretMotionMagicPositionRelative(-limelight.getTx() + offsetAngleDeg);
            resetExtraOneTimer();
            startExtraOneTimeout(0.1);
        }
        else {
            targetFound = false;
        }
    }

    @Override
    public boolean isFinished() {
        if (!targetFound) {
            return true;
        }
        else if (isExtraOneTimedOut() && turret.hasFinishedTrajectory()) {
            System.out.println("Limelight turret index angle = finished");
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Turret interrupted = " + interrupted);
    }
}
