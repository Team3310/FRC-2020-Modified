package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;


public class HoodSetCachedLimelightOffset extends CommandBase {
    private final Shooter shooter;
    private double offsetAngle;

    public HoodSetCachedLimelightOffset(Shooter shooter, double offsetAngle) {
        this.shooter = shooter;
        this.offsetAngle = offsetAngle;
    }

    @Override
    public void initialize() {
        shooter.setCachedLimelightHoodOffset(offsetAngle);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
