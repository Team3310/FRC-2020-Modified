package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;


public class LimelightSetPipeline extends CommandBase {
    private final Limelight limelight;
    private int pipeline;

    public LimelightSetPipeline(Limelight limelight, int pipeline) {
        this.limelight = limelight;
        this.pipeline = pipeline;
        addRequirements(limelight);
    }

    @Override
    public void initialize() {
        limelight.setPipeline(pipeline);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
