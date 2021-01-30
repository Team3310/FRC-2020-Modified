package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.MagazineIndexDividerToTurret;
import frc.robot.utilities.Util;

public class Magazine extends SubsystemBase {

    public static enum MagazineControlMode {
        MOTION_MAGIC, MOTION_MAGIC_TRACK_TURRET, VELOCITY, MANUAL
    };

    // Conversions
    private static final double MAGAZINE_OUTPUT_TO_ENCODER_RATIO = 500.0 / 11.0;
    public static final double MAGAZINE_REVOLUTIONS_TO_ENCODER_TICKS = MAGAZINE_OUTPUT_TO_ENCODER_RATIO * Constants.ENCODER_TICKS_PER_MOTOR_REVOLUTION;
    public static final double MAGAZINE_DEGREES_TO_ENCODER_TICKS = MAGAZINE_REVOLUTIONS_TO_ENCODER_TICKS / 360.0;

    // Motor Controllers
    private TalonFX magMotor;

    // Misc
    private static final int kMagazineVelocitySlot = 0;
    private static final int kMagazineMotionMagicSlot = 1;
    private double homePositionAngleDegrees = Constants.MAGAZINE_COMPETITION_HOME_POSITION_DEGREES;
    private double targetPositionTicks = 0;

    private MagazineControlMode magazineControlMode;

    private final static Magazine INSTANCE = new Magazine();

    private Magazine() {
        magMotor = new TalonFX(Constants.MAGAZINE_MOTOR_CAN_ID);

        TalonFXConfiguration configs = new TalonFXConfiguration();
        configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        configs.closedloopRamp = 1;
        magMotor.configAllSettings(configs);

        magMotor.setInverted(TalonFXInvertType.Clockwise);
        magMotor.setNeutralMode(NeutralMode.Brake);
        magMotor.configMotionCruiseVelocity(6000);
        magMotor.configMotionAcceleration(14000);
        magMotor.configMotionSCurveStrength(4);

        final StatorCurrentLimitConfiguration statorCurrentConfigs = new StatorCurrentLimitConfiguration();
        statorCurrentConfigs.currentLimit = 40;
        statorCurrentConfigs.triggerThresholdCurrent = 60;
        statorCurrentConfigs.triggerThresholdTime = 0.5;
        statorCurrentConfigs.enable = true;
        magMotor.configStatorCurrentLimit(statorCurrentConfigs);

        magMotor.config_kF(kMagazineVelocitySlot, 0.040);
        magMotor.config_kP(kMagazineVelocitySlot, 0.05);//0.05
        magMotor.config_kI(kMagazineVelocitySlot, 0.0);//0.0001
        magMotor.config_kD(kMagazineVelocitySlot, 0.0);
        magMotor.config_IntegralZone(kMagazineVelocitySlot, (int)this.MagazineRPMToNativeUnits(10));

        magMotor.config_kF(kMagazineMotionMagicSlot, 0.03);
        magMotor.config_kP(kMagazineMotionMagicSlot, 0.05);//0.05
        magMotor.config_kI(kMagazineMotionMagicSlot, 0.00001);//0.0001
        magMotor.config_kD(kMagazineMotionMagicSlot, 0.0);
        magMotor.config_IntegralZone(kMagazineMotionMagicSlot, (int)this.MagazineRPMToNativeUnits(10));
    }

    public static Magazine getInstance() {
        return INSTANCE;
    }


    private synchronized void setMagazineControlMode(MagazineControlMode controlMode) {
        this.magazineControlMode = controlMode;
    }

    private synchronized MagazineControlMode getTurretControlMode() {
        return this.magazineControlMode;
    }

    public void setMagazineSpeed(double speed) {
        setMagazineControlMode(MagazineControlMode.MANUAL);
        this.magMotor.set(ControlMode.PercentOutput, speed);
        System.out.println("Set Magazine Speed");
    }

    public void resetHomePosition() {
        this.magMotor.setSelectedSensorPosition(0);
    }

    public double getMagazineRPM() {
        return magMotor.getSelectedSensorVelocity() / MAGAZINE_REVOLUTIONS_TO_ENCODER_TICKS * 10.0 * 60.0;
    }
    public double getMagazineRotations() {
        return magMotor.getSelectedSensorPosition() / Constants.ENCODER_TICKS_PER_MOTOR_REVOLUTION / MAGAZINE_OUTPUT_TO_ENCODER_RATIO;
    }
    public void setMagazineRPM(double rpm) {
        setMagazineControlMode(MagazineControlMode.VELOCITY);
        this.magMotor.set(ControlMode.Velocity, this.MagazineRPMToNativeUnits(rpm));
    }

    public double getMagazineAngleAbsoluteDegrees() {
        return (double)magMotor.getSelectedSensorPosition() / MAGAZINE_DEGREES_TO_ENCODER_TICKS + homePositionAngleDegrees;
    }

    // Motion Magic
    public synchronized void setMagazineMotionMagicPositionAbsolute(double angle) {
        setMagazineControlMode(MagazineControlMode.MOTION_MAGIC);
        magMotor.selectProfileSlot(kMagazineMotionMagicSlot, 0);
        targetPositionTicks = getMagazineEncoderTicksAbsolute(angle);
        magMotor.set(ControlMode.MotionMagic, targetPositionTicks, DemandType.ArbitraryFeedForward, 0.04);
    }

    public synchronized void setMagazineMotionMagicPositionAbsoluteInternal(double angle) {
        magMotor.selectProfileSlot(kMagazineMotionMagicSlot, 0);
        targetPositionTicks = getMagazineEncoderTicksAbsolute(angle);
        magMotor.set(ControlMode.MotionMagic, targetPositionTicks, DemandType.ArbitraryFeedForward, 0.04);
    }

    public synchronized boolean hasFinishedTrajectory() {
        return Util.epsilonEquals(magMotor.getActiveTrajectoryPosition(), targetPositionTicks, 200);
    }

    private int getMagazineEncoderTicksAbsolute(double angle) {
        double positionDegrees = angle - homePositionAngleDegrees;
        return (int) (positionDegrees * MAGAZINE_DEGREES_TO_ENCODER_TICKS);
    }

    public double MagazineRPMToNativeUnits(double rpm) {
        return rpm * MAGAZINE_REVOLUTIONS_TO_ENCODER_TICKS / 10.0D / 60.0D;
    }

    public double getStatorCurrent() {
        return magMotor.getStatorCurrent();
    }

    public void setTurretTrackMode() {
        setMagazineControlMode(MagazineControlMode.MOTION_MAGIC_TRACK_TURRET);
        updateTurretTrack();
    }

    private void updateTurretTrack() {
        double magazineAngle = MagazineIndexDividerToTurret.closestDividerDeltaAngle(Turret.getInstance().getTurretAngleAbsoluteDegrees(), getMagazineAngleAbsoluteDegrees());
        setMagazineMotionMagicPositionAbsoluteInternal(magazineAngle);
    }

    public void periodic() {
        if (getTurretControlMode() == MagazineControlMode.MOTION_MAGIC_TRACK_TURRET) {
            updateTurretTrack();
        }
 //       SmartDashboard.putNumber("Magazine RPM", this.getMagazineRPM());
 //       SmartDashboard.putNumber("Magazine Roller Rotations", this.getMagazineRotations());
 //       SmartDashboard.putNumber("Magazine Current", magMotor.getStatorCurrent());
        SmartDashboard.putNumber("Magazine Angle", getMagazineAngleAbsoluteDegrees());
    }
}

