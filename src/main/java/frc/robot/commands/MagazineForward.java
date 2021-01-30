package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Magazine;

public class MagazineForward extends SequentialCommandGroup {

    public MagazineForward(Intake intake, Magazine magazine) {

        addCommands(

                new ParallelCommandGroup(

                        // Intake RPM
                        new IntakeSetRPM(intake, Constants.INTAKE_SLOW_RPM),

                        // Magazine CounterClockwise
                        new MagazineSetRPMLimit(magazine, Constants.MAGAZINE_FORWARD_RPM, Constants.MAGAZINE_JAM_STATOR_CURRENT)
                )
        );
    }
}