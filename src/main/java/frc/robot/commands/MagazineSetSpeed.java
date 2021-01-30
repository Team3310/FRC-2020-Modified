package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Magazine;


public class MagazineSetSpeed extends CommandBase {
    private final Magazine magazine;
    private double speed;

    public MagazineSetSpeed(Magazine magazine, double speed) {
        this.magazine = magazine;
        this.speed = speed;
        addRequirements(magazine);
    }

    @Override
    public void initialize() {
        magazine.setMagazineSpeed(speed);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
