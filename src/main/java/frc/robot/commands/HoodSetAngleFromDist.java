package frc.robot.commands;

import frc.robot.subsystems.Shooter;


public class HoodSetAngleFromDist extends ExtraTimeoutCommand {
    private final Shooter shooter;
    private double distance;

    public HoodSetAngleFromDist(Shooter shooter, double distance) {
        this.shooter = shooter;
        this.distance = distance;
    }

    @Override
    public void initialize() {
        shooter.setHoodAngleBasedOnDistance(distance);
        resetExtraOneTimer();
        startExtraOneTimeout(0.1);
    }

    @Override
    public boolean isFinished() {
        if (isExtraOneTimedOut() && shooter.hasFinishedHoodTrajectory()) {
            return true;
        }
        return false;
    }
}
