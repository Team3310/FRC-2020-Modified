package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Turret;

public class ClimbSetSpeedUp extends SequentialCommandGroup {

    public ClimbSetSpeedUp(Intake intake, Turret turret, Magazine magazine) {
        addCommands(
                new IntakeSetSpeed(intake, 0.5)
        );
    }
}