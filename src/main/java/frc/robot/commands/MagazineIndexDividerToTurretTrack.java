package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Magazine;


public class MagazineIndexDividerToTurretTrack extends CommandBase {
    private final Magazine magazine;

    public MagazineIndexDividerToTurretTrack(Magazine magazine) {
        this.magazine = magazine;
        addRequirements(magazine);
    }

    @Override
    public void initialize() {
        magazine.setTurretTrackMode();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
