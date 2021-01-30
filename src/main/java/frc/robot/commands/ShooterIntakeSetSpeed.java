package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;


public class ShooterIntakeSetSpeed extends CommandBase {
    private final Shooter shooter;
    private double speed;

    public ShooterIntakeSetSpeed(Shooter shooter, double speed) {
        this.shooter = shooter;
        this.speed = speed;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setIntakeSpeed(speed);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
