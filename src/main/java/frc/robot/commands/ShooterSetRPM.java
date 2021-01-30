package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.Util;


public class ShooterSetRPM extends CommandBase {
    private final Shooter shooter;
    private double shooterMainRPM;
    private double shooterKickerRPM;


    public ShooterSetRPM(Shooter shooter, double shooterMainRPM, double shooterKickerRPM) {
        this.shooter = shooter;
        this.shooterMainRPM = shooterMainRPM;
        this.shooterKickerRPM = shooterKickerRPM;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setMainRPM(shooterMainRPM);
        shooter.setKickerRPM(shooterKickerRPM);
    }

    @Override
    public boolean isFinished() {
        boolean isMainUpToSpeed = Util.epsilonEquals(shooter.getMainRPM(), shooterMainRPM, Constants.SHOOTER_MAIN_RPM_EPSILON);
        boolean isKickerUpToSpeed = Util.epsilonEquals(shooter.getKickerRPM(), shooterKickerRPM, Constants.SHOOTER_KICKER_RPM_EPSILON);
        return isMainUpToSpeed && isKickerUpToSpeed;
    }
}
