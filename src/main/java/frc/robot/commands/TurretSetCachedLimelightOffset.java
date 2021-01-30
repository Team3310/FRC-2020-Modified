package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;


public class TurretSetCachedLimelightOffset extends CommandBase {
    private final Turret turret;
    private double offsetAngle;

    public TurretSetCachedLimelightOffset(Turret turret, double offsetAngle) {
        this.turret = turret;
        this.offsetAngle = offsetAngle;
    }

    @Override
    public void initialize() {
        turret.setCachedLimelightTurretOffset(offsetAngle);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
