package frc.robot.auto.commands;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.*;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class ShooterAutoMediumShotTrack extends SequentialCommandGroup {

    public ShooterAutoMediumShotTrack(Shooter shooter, Magazine magazine, Turret turret, double magazineRotations) {
        addCommands(
                new InstantCommand(()-> Limelight.getInstance().setPipeline(Constants.LIMELIGHT_MEDIUM_PIPELINE)),
                new LimelightSetLED(Limelight.getInstance(), Limelight.LightMode.ON),
                new ShooterSetRPM(shooter, Constants.SHOOTER_MAIN_MEDIUM_RPM, Constants.SHOOTER_KICKER_MEDIUM_RPM),
                new HoodSetAngle(shooter, Constants.HOOD_MEDIUM_ANGLE_DEGREES),
                new SequentialCommandGroup(
                        new TurretSetToGyroAngle(turret, Constants.TURRET_GYRO_OFFSET_MEDIUM_SHOT_ANGLE_DEGREES),
                        new TurretSetToTrackLimelightAngle(turret, Constants.LIMELIGHT_OFFSET_MEDIUM_SHOT_DEGREES, Constants.TURRET_GYRO_OFFSET_MEDIUM_SHOT_ANGLE_DEGREES)
                )
        );
    }
}