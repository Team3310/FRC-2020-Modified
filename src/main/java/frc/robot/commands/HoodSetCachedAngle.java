package frc.robot.commands;

import frc.robot.subsystems.Shooter;


public class HoodSetCachedAngle extends ExtraTimeoutCommand {
    private final Shooter shooter;

    public HoodSetCachedAngle(Shooter shooter) {
        this.shooter = shooter;
    }

    @Override
    public void initialize() {
        shooter.setHoodMotionMagicPositionAbsolute(shooter.getCachedHoodAngle());
        resetExtraOneTimer();
        startExtraOneTimeout(0.1);
    }

    @Override
    public boolean isFinished() {
        if (isExtraOneTimedOut() && shooter.hasFinishedHoodTrajectory()) {
            System.out.println("hood angle = finished");
            return true;
        }
        return false;
    }
}
