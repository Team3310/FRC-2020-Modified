package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;


public class LimelightSetLED extends CommandBase {
    private final Limelight limelight;
    private Limelight.LightMode lightMode;

    public LimelightSetLED(Limelight limelight, Limelight.LightMode lightMode) {
        this.limelight = limelight;
        this.lightMode = lightMode;
        addRequirements(limelight);
    }

    @Override
    public void initialize() {
        limelight.setLedMode(lightMode);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
