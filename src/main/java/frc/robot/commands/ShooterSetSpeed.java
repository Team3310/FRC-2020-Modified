package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;


public class ShooterSetSpeed extends CommandBase {
    private final Shooter shooter;
    private double mainSpeed;
    private double kickerSpeed;


    public ShooterSetSpeed(Shooter shooter, double mainSpeed, double kickerSpeed) {
        this.shooter = shooter;
        this.mainSpeed = mainSpeed;
        this.kickerSpeed = kickerSpeed;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setMainSpeed(mainSpeed);
        shooter.setKickerSpeed(kickerSpeed);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
