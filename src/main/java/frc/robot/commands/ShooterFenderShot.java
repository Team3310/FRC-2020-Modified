package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class ShooterFenderShot extends SequentialCommandGroup {

    public ShooterFenderShot(Shooter shooter, Magazine magazine, Turret turret) {
        addCommands(
                new ShooterSetReady(shooter,false),
                new ShooterSetRPM(shooter, Constants.SHOOTER_MAIN_FENDER_RPM, Constants.SHOOTER_KICKER_FENDER_RPM),
                new SequentialCommandGroup(
                        new TurretSetToGyroAngle(turret, Constants.TURRET_GYRO_OFFSET_FENDER_SHOT_ANGLE_DEGREES)
//                        new MagazineIndexDividerToTurretTrack(magazine)
                ),
                new ShooterSetCachedHoodAngle(shooter, Constants.HOOD_FENDER_ANGLE_DEGREES),
                new TurretSetCachedLimelightOffset(turret, Constants.LIMELIGHT_OFFSET_FENDER_SHOT_DEGREES),
                new LimelightSetLED(Limelight.getInstance(), Limelight.LightMode.ON),
                new ShooterSetReady(shooter, true)
        );
    }
}