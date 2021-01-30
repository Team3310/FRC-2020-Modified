package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class ShooterShoot extends SequentialCommandGroup {

    public ShooterShoot(Shooter shooter, Magazine magazine, Turret turret, Limelight limelight) {
        addCommands(
 //           new HoodSetCachedAngle(shooter),
 //           new ShooterIsReady(shooter),
 //           new TurretSetToCachedLimelightAngle(turret),
 //           new MagazineIndexDividerToTurret(magazine, turret),
            new ShooterIntakeSetRPM(shooter, Constants.SHOOTER_INTAKE_RPM)
 //           new MagazineSetRPMLimit(magazine, Constants.MAGAZINE_SHOOT_RPM, Constants.MAGAZINE_JAM_STATOR_CURRENT)
        );
    }
}