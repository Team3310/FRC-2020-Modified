package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Magazine;


public class MagazineSetRPM extends CommandBase {
    private final Magazine magazine;
    private double rpm;
    private double degreesRotation;

    public MagazineSetRPM(Magazine magazine, double rpm) {
        this.magazine = magazine;
        this.rpm = rpm;
        addRequirements(magazine);
    }

    @Override
    public void initialize() {
        magazine.setMagazineRPM(rpm);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
