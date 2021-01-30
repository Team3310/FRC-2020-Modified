package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;


public class ShooterSetCachedHoodAngle extends CommandBase {
    private final Shooter shooter;
    private double hoodAngle;

    public ShooterSetCachedHoodAngle(Shooter shooter, double hoodAngle) {
        this.shooter = shooter;
        this.hoodAngle = hoodAngle;
    }

    @Override
    public void initialize() {
        shooter.setCachedHoodAngle(hoodAngle);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
