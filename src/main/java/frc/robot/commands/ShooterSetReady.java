package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;


public class ShooterSetReady extends CommandBase {
    private final Shooter shooter;
    private boolean isReady;

    public ShooterSetReady(Shooter shooter, boolean isReady) {
        this.shooter = shooter;
        this.isReady = isReady;
    }

    @Override
    public void initialize() {
        shooter.setReady(isReady);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
