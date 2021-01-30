package frc.robot.commands;

import frc.robot.subsystems.Turret;

public class TurretSetToDriveShootOffsetAngle extends ExtraTimeoutCommand {
    private final Turret turret;

    public TurretSetToDriveShootOffsetAngle(Turret subsystem) {
        this.turret = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        turret.setTurretMotionMagicPositionAbsolute(Turret.getInstance().getDriveShootOffSetAngle());
        resetExtraOneTimer();
        startExtraOneTimeout(0.1);
    }

    @Override
    public boolean isFinished() {
        if (isExtraOneTimedOut() && turret.hasFinishedTrajectory()) {
            return true;
        }
        return false;
    }
}

