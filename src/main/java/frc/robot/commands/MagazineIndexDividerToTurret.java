package frc.robot.commands;

import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Turret;
import frc.robot.utilities.Util;


public class MagazineIndexDividerToTurret extends ExtraTimeoutCommand {
    private final Magazine magazine;
    private final Turret turret;

    public MagazineIndexDividerToTurret(Magazine magazine, Turret turret) {
        this.magazine = magazine;
        this.turret = turret;
        addRequirements(magazine);
    }

    @Override
    public void initialize() {
        double magazineAngle = closestDividerDeltaAngle(turret.getTurretAngleAbsoluteDegrees(), magazine.getMagazineAngleAbsoluteDegrees());
        System.out.println("Magazine index angle = " + magazineAngle);
        magazine.setMagazineMotionMagicPositionAbsolute(magazineAngle);
        resetExtraOneTimer();
        startExtraOneTimeout(0.1);
    }

    @Override
    public boolean isFinished() {
        if (isExtraOneTimedOut() && magazine.hasFinishedTrajectory()) {
            System.out.println("Magazine index angle = finished");
            return true;
        }
        return false;
    }

    public static double closestDividerDeltaAngle(double turretAngle, double magazineAngle) {
        final double dividerDeltaAngle = 72.0;
        System.out.println("closestDividerDeltaAngle magazineAngle = " + magazineAngle);
        double normalizedDividerAngle = Util.normalizeAngle90ToMinus270(magazineAngle);
        System.out.println("normalizedDividerAngle = " + normalizedDividerAngle + ", magazineAngle = " + magazineAngle);
        double currentDividerAngle = normalizedDividerAngle;
        double closestDividerDeltaAngle = turretAngle - currentDividerAngle;
        for (int i = 1; i < 5; i++) {
            currentDividerAngle = Util.normalizeAngle90ToMinus270(currentDividerAngle + dividerDeltaAngle);
            double currentDividerDeltaAngle = turretAngle - currentDividerAngle;
            if (Math.abs(currentDividerDeltaAngle) < Math.abs(closestDividerDeltaAngle)) {
                closestDividerDeltaAngle = currentDividerDeltaAngle;
            }
        }

        return magazineAngle + closestDividerDeltaAngle;
    }
}
