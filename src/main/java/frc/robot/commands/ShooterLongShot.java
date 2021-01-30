package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class ShooterLongShot extends SequentialCommandGroup {

    public ShooterLongShot(Shooter shooter, Magazine magazine, Turret turret) {
        addCommands(
                new InstantCommand(()->Limelight.getInstance().setPipeline(Constants.LIMELIGHT_LONG_PIPELINE)),
//                new ShooterSetReady(shooter, false),
                new ShooterSetRPM(shooter, Constants.SHOOTER_MAIN_LONG_RPM, Constants.SHOOTER_KICKER_LONG_RPM),
                new SequentialCommandGroup(
                        new TurretSetToGyroAngle(turret, Constants.TURRET_GYRO_OFFSET_LONG_SHOT_ANGLE_DEGREES)
//                        new MagazineIndexDividerToTurretTrack(magazine)
                ),
                new ShooterSetCachedHoodAngle(shooter, Constants.HOOD_LONG_ANGLE_DEGREES),
                new TurretSetCachedLimelightOffset(turret, Constants.LIMELIGHT_OFFSET_LONG_SHOT_DEGREES),
                new LimelightSetLED(Limelight.getInstance(), Limelight.LightMode.ON)
//                new ShooterSetReady(shooter, true)
        );
    }
}