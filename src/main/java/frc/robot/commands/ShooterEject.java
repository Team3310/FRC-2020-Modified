package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Shooter;

public class ShooterEject extends SequentialCommandGroup {

    public ShooterEject(Shooter shooter) {
        addCommands(
                new ShooterSetSpeed(shooter, -0.3, -0.3),
                new ShooterIntakeSetSpeed(shooter, -0.3)
         );
    }
}