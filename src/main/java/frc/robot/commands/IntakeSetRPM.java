package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;


public class IntakeSetRPM extends CommandBase {
    private final Intake intake;
    private double rpm;

    public IntakeSetRPM(Intake intake, double rpm) {
        this.intake = intake;
        this.rpm = rpm;
        addRequirements(this.intake);
    }

    @Override
    public void initialize() {
        intake.setRollerRPM(rpm);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
