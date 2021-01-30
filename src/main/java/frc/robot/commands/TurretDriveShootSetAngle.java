package frc.robot.commands;

import frc.robot.subsystems.Turret;

public class TurretDriveShootSetAngle extends ExtraTimeoutCommand {
    private final Turret turret;
    private double angle;

    public TurretDriveShootSetAngle(Turret subsystem, double angle) {
        this.turret = subsystem;
        this.angle = Turret.getInstance().getDriveShootOffSetAngle();
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        turret.setTurretMotionMagicPositionAbsolute(angle);
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
