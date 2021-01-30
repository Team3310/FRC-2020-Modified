package frc.robot.auto.commands;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.*;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class ShooterAutoShotTrack extends SequentialCommandGroup {

    public ShooterAutoShotTrack(Shooter shooter, Magazine magazine, Turret turret, double magazineRotations) {
        addCommands(
                new ParallelCommandGroup(
                        new ShooterSetRPM(shooter, Constants.SHOOTER_MAIN_AUTO_RPM, Constants.SHOOTER_KICKER_AUTO_RPM),
                        new TurretSetAngle(turret, Constants.TURRET_COMPETITION_HOME_POSITION_DEGREES),
                        new HoodSetAngle(shooter, Constants.HOOD_AUTO_ANGLE_DEGREES)
                ),
                new TurretSetToLimelightAngle(turret, Constants.LIMELIGHT_OFFSET_AUTO_SHOT_DEGREES),
                new ShooterIntakeSetRPM(shooter, Constants.SHOOTER_INTAKE_RPM),
                new MagazineSetRPMRotations(magazine, Constants.MAGAZINE_SHOOT_AUTO_RPM,
                        magazineRotations)
        );
    }
}