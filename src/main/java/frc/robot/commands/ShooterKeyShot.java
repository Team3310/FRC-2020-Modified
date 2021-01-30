package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class ShooterKeyShot extends ParallelCommandGroup {

    public ShooterKeyShot(Shooter shooter, Magazine magazine, Turret turret) {
        addCommands(
//                new ShooterSetReady(shooter,false),
                new ShooterSetRPM(shooter, Constants.SHOOTER_MAIN_KEY_RPM, Constants.SHOOTER_KICKER_KEY_RPM),
                new SequentialCommandGroup(
                        new TurretSetToGyroAngle(turret, Constants.TURRET_GYRO_OFFSET_KEY_SHOT_ANGLE_DEGREES)
 //                       new MagazineIndexDividerToTurretTrack(magazine)
                ),
                new ShooterSetCachedHoodAngle(shooter, Constants.HOOD_KEY_ANGLE_DEGREES),
                new TurretSetCachedLimelightOffset(turret, Constants.LIMELIGHT_OFFSET_KEY_SHOT_DEGREES),
                new LimelightSetLED(Limelight.getInstance(), Limelight.LightMode.ON)
//                new ShooterSetReady(shooter, true)
        );
    }
}