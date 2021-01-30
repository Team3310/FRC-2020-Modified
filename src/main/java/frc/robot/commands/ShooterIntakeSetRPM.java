package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.Util;


public class ShooterIntakeSetRPM extends CommandBase {
    private final Shooter shooter;
    private double rpm;

    public ShooterIntakeSetRPM(Shooter shooter, double rpm) {
        this.shooter = shooter;
        this.rpm = rpm;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.seIntakeRPM(rpm);
    }

    @Override
    public boolean isFinished() {
        boolean isIntakeUpToSpeed = Util.epsilonEquals(shooter.getIntakeRPM(), rpm, Constants.SHOOTER_MAIN_RPM_EPSILON);
        return isIntakeUpToSpeed;
    }
}
