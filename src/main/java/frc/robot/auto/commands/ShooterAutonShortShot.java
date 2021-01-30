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

public class ShooterAutonShortShot extends ParallelCommandGroup {

    public ShooterAutonShortShot(Shooter shooter, Magazine magazine, Turret turret) {
        addCommands(
                new InstantCommand(()->Limelight.getInstance().setPipeline(Constants.LIMELIGHT_AUTO_PIPELINE)),
//                new ShooterSetReady(shooter,false),
                new ShooterSetRPM(shooter, Constants.SHOOTER_MAIN_AUTON_SHORT_RPM, Constants.SHOOTER_KICKER_AUTON_SHORT_RPM),
                new SequentialCommandGroup(
                        new TurretSetToTrackGyroAngle(turret, Constants.TURRET_GYRO_OFFSET_AUTON_SHORT_SHOT_ANGLE_DEGREES),
                        new MagazineIndexDividerToTurretTrack(magazine)
                ),
                new ShooterSetCachedHoodAngle(shooter, Constants.HOOD_AUTON_SHORT_ANGLE_DEGREES),
                new TurretSetCachedLimelightOffset(turret, Constants.LIMELIGHT_OFFSET_AUTON_SHORT_SHOT_DEGREES),
                new LimelightSetLED(Limelight.getInstance(), Limelight.LightMode.ON)
//                new ShooterSetReady(shooter, true)
        );

    }
}