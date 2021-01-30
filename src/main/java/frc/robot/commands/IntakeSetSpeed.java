package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;


public class IntakeSetSpeed extends CommandBase {
    private final Intake intake;
    private double speed;

    public IntakeSetSpeed(Intake intake, double speed) {
        this.intake = intake;
        this.speed = speed;
    }

    @Override
    public void initialize() {
        intake.setRollerSpeed(speed);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
