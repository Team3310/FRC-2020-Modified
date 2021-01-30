package frc.robot.commands;

import frc.robot.subsystems.Magazine;


public class MagazineSetRPMLimit extends ExtraTimeoutCommand {
    private final Magazine magazine;
    private double rpm;
    private double statorCurrentLimit;
    private boolean isJamDetected = false;

    public MagazineSetRPMLimit(Magazine magazine, double rpm, double statorCurrentLimit) {
        this.magazine = magazine;
        this.rpm = rpm;
        this.statorCurrentLimit = statorCurrentLimit;
        addRequirements(magazine);
    }

    @Override
    public void initialize() {
        magazine.setMagazineRPM(rpm);
        resetExtraOneTimer();
        startExtraOneTimeout(2.0);
    }

    @Override
    public boolean isFinished() {
        if (isExtraOneTimedOut() && isExtraTwoTimedOut()) {
            magazine.setMagazineRPM(rpm);
            resetExtraOneTimer();
            startExtraOneTimeout(2.0);
            isJamDetected = false;
            return false;
        }
        else if (isExtraOneTimedOut() && isJamDetected == false && magazine.getStatorCurrent() > statorCurrentLimit) {
            System.out.println("Stator current exceeded = " + magazine.getStatorCurrent() + ", timeout = " + timeSinceExtraOneInitialized());
            magazine.setMagazineRPM(-rpm);
            resetExtraTwoTimer();
            startExtraTwoTimeout(1.0);
            isJamDetected = true;
            return false;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        magazine.setMagazineSpeed(0);
        System.out.println("Stator current limit end, interupted = " + interrupted);
    }
}
