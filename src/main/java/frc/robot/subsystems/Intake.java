package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utilities.Util;

public class Intake extends SubsystemBase {

    // Conversions
    private static final double INTAKE_ROLLER_OUTPUT_TO_ENCODER_RATIO = 30.0 / 12.0;
    public static final double INTAKE_ROLLER_REVOLUTIONS_TO_ENCODER_TICKS = INTAKE_ROLLER_OUTPUT_TO_ENCODER_RATIO * Constants.ENCODER_TICKS_PER_MOTOR_REVOLUTION;
    private static final double CLIMB_OUTPUT_TO_ENCODER_RATIO = (50.0 / 12.0) * (50.0 / 20.0);
    public static final double CLIMB_DRUM_DIAMETER_INCHES = 0.625;
    public static final double CLIMB_REVOLUTIONS_TO_INCHES = Math.PI * CLIMB_DRUM_DIAMETER_INCHES;
    public static final double CLIMB_INCHES_TO_ENCODER_TICKS = CLIMB_OUTPUT_TO_ENCODER_RATIO * Constants.ENCODER_TICKS_PER_MOTOR_REVOLUTION / CLIMB_REVOLUTIONS_TO_INCHES;


    // Motor Controllers
    private TalonFX intakeMotor;

    // Misc
    private static final int kIntakeVelocitySlot = 0;
    private static final int kClimbMotionMagicSlot = 1;
    private double targetPositionTicks = 0;

    private final static Intake INSTANCE = new Intake();

    private Intake() {
        intakeMotor = new TalonFX(Constants.INTAKE_MOTOR_CAN_ID);

        TalonFXConfiguration configs = new TalonFXConfiguration();
        configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        intakeMotor.configAllSettings(configs);

        intakeMotor.setInverted(TalonFXInvertType.CounterClockwise);
        intakeMotor.setNeutralMode(NeutralMode.Brake);
        intakeMotor.configMotionCruiseVelocity(6000);
        intakeMotor.configMotionAcceleration(14000);
        intakeMotor.configMotionSCurveStrength(4);

        final StatorCurrentLimitConfiguration statorCurrentConfigs = new StatorCurrentLimitConfiguration();
        statorCurrentConfigs.currentLimit = 40;
        statorCurrentConfigs.enable = true;
        intakeMotor.configStatorCurrentLimit(statorCurrentConfigs);

        intakeMotor.config_kF(kIntakeVelocitySlot, 0.055);
        intakeMotor.config_kP(kIntakeVelocitySlot, 0.10);
        intakeMotor.config_kI(kIntakeVelocitySlot, 0.0001);
        intakeMotor.config_kD(kIntakeVelocitySlot, 0.0);
        intakeMotor.config_IntegralZone(kIntakeVelocitySlot, (int)this.RollerRPMToNativeUnits(200));

        intakeMotor.config_kF(kClimbMotionMagicSlot, 0.0);
        intakeMotor.config_kP(kClimbMotionMagicSlot, 0.0);
        intakeMotor.config_kI(kClimbMotionMagicSlot, 0.0);
        intakeMotor.config_kD(kClimbMotionMagicSlot, 0.0);
        intakeMotor.config_IntegralZone(kClimbMotionMagicSlot, (int)this.RollerRPMToNativeUnits(200));
    }

    public static Intake getInstance() {
        return INSTANCE;
    }

    public void setRollerSpeed(double speed) {
        this.intakeMotor.set(ControlMode.PercentOutput, speed);
        System.out.println("Set Intake Speed");
    }

    // Motion Magic
    public synchronized void setClimbMotionMagicPositionAbsolute(double inches) {
        intakeMotor.selectProfileSlot(kClimbMotionMagicSlot, 0);
        targetPositionTicks = getClimbEncoderTicksAbsolute(limitClimbInches(inches));
        intakeMotor.set(ControlMode.MotionMagic, targetPositionTicks, DemandType.ArbitraryFeedForward, 0.04);
    }

    public synchronized boolean hasFinishedTrajectory() {
        return Util.epsilonEquals(intakeMotor.getActiveTrajectoryPosition(), targetPositionTicks, 100);
    }

    private int getClimbEncoderTicksAbsolute(double inches) {
        return (int) (inches * CLIMB_INCHES_TO_ENCODER_TICKS);
    }

    public double getClimbAbsoluteInches() {
        return (double)intakeMotor.getSelectedSensorPosition() / CLIMB_INCHES_TO_ENCODER_TICKS;
    }

    private double limitClimbInches(double targetInches) {
        if (targetInches < Constants.CLIMB_MIN_INCHES) {
            return Constants.CLIMB_MIN_INCHES;
        } else if (targetInches > Constants.CLIMB_MAX_INCHES) {
            return Constants.CLIMB_MAX_INCHES;
        }

        return targetInches;
    }

    public void resetIntakeEncoder() {
        this.intakeMotor.setSelectedSensorPosition(0);
    }

    public double getRollerRotations() {
        return intakeMotor.getSelectedSensorPosition() / Constants.ENCODER_TICKS_PER_MOTOR_REVOLUTION / INTAKE_ROLLER_OUTPUT_TO_ENCODER_RATIO;
    }

    public double getClimbRotations() {
        return intakeMotor.getSelectedSensorPosition() / Constants.ENCODER_TICKS_PER_MOTOR_REVOLUTION / CLIMB_OUTPUT_TO_ENCODER_RATIO;
    }

    public double getClimbInches() {
        return getClimbRotations() * CLIMB_REVOLUTIONS_TO_INCHES;
    }

    public double getRollerRPM() {
        return intakeMotor.getSelectedSensorVelocity() / INTAKE_ROLLER_REVOLUTIONS_TO_ENCODER_TICKS * 10.0 * 60.0;
    }

    public void setRollerRPM(double rpm) {
        this.intakeMotor.set(ControlMode.Velocity, this.RollerRPMToNativeUnits(rpm));
    }

    public double RollerRPMToNativeUnits(double rpm) {
        return rpm * INTAKE_ROLLER_REVOLUTIONS_TO_ENCODER_TICKS / 10.0D / 60.0D;
    }

//        SmartDashboard.putNumber("Intake Roller Rotations", this.getRollerRotations());
//        SmartDashboard.putNumber("Intake Roller RPM", this.getRollerRPM());
//        SmartDashboard.putNumber("Climb Inches", this.getClimbInches());
}

