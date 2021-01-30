package frc.robot.auto.commands;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.MagazineSetRPMRotations;
import frc.robot.commands.ShooterIntakeSetRPM;
import frc.robot.commands.ShooterIntakeSetSpeed;
import frc.robot.commands.TurretSetToLimelightAngle;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class ShooterAutoDoubleShoot extends SequentialCommandGroup {

    public ShooterAutoDoubleShoot(Shooter shooter, Magazine magazine, Turret turret, double magazineRotations) {
        addCommands(
                new TurretSetToLimelightAngle(turret, Constants.LIMELIGHT_OFFSET_AUTO_DOUBLE_SHOT_DEGREES),
                new ShooterIntakeSetRPM(shooter, Constants.SHOOTER_INTAKE_RPM),
                new WaitCommand(2),
                new ShooterIntakeSetSpeed(shooter, 0)

        );
    }
}