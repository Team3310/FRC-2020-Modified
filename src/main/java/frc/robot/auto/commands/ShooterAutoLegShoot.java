package frc.robot.auto.commands;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.*;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class ShooterAutoLegShoot extends SequentialCommandGroup {

    public ShooterAutoLegShoot(Shooter shooter, Magazine magazine, Turret turret, double magazineRotations) {
        addCommands(
                new WaitCommand(0.25),
                new MagazineSetRPM(magazine, Constants.MAGAZINE_SHOOT_RPM),
                new TurretSetToLimelightAngle(turret, Constants.LIMELIGHT_OFFSET_LEG_SHOT_DEGREES),
                new ShooterIntakeSetRPM(shooter, Constants.SHOOTER_INTAKE_RPM),
                new WaitCommand(2.2),
                new ShooterIntakeSetSpeed(shooter, 0)
 //               new MagazineSetRPMRotations(magazine, Constants.MAGAZINE_SHOOT_AUTO_LEG_RPM,
 //                       magazineRotations)
        );
    }
}