package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Magazine;


public class MagazineSetRPMRotations extends CommandBase {
    private final Magazine magazine;
    private double rpm;
    private double degreesRotation;
    private double magazineAngle;

    public MagazineSetRPMRotations(Magazine magazine, double rpm, double degreesRotation) {
        this.magazine = magazine;
        this.rpm = rpm;
        this.degreesRotation = degreesRotation;
        addRequirements(magazine);
    }

    @Override
    public void initialize() {
        magazineAngle = magazine.getMagazineAngleAbsoluteDegrees() + degreesRotation;
        magazine.setMagazineRPM(rpm);
    }

    @Override
    public boolean isFinished() {
        if (magazine.getMagazineAngleAbsoluteDegrees() > magazineAngle) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
//        magazine.setMagazineRPM(Constants.MAGAZINE_INTAKE_RPM);
        magazine.setMagazineSpeed(0);

    }
}
