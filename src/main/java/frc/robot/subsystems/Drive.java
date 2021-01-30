package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.TurretSetLagAngle;
import frc.robot.controller.GameController;

public class Drive extends SubsystemBase {

    public static enum DriveControlMode {
        JOYSTICK, PATH_FOLLOWING
    }

    private DriveControlMode driveControlMode = DriveControlMode.JOYSTICK;

    private static final double DRIVE_OUTPUT_TO_ENCODER_RATIO = 50.0 / 11.0;

    // Speed Control
    private static final double STEER_NON_LINEARITY = 0.5;
    private static final double MOVE_NON_LINEARITY = 1.0;

    private static final int MOVE_NON_LINEAR = 0;
    private static final int STEER_NON_LINEAR = -3;

    private static final double MOVE_SCALE = 1.0;
    private static final double STEER_SCALE = 0.75;

    private static final double MOVE_TRIM = 0.0;
    private static final double STEER_TRIM = 0.0;

    private static final double STICK_DEADBAND = 0.02;

    public static final double OPEN_LOOP_PERCENT_OUTPUT_LO = 0.6;
    public static final double OPEN_LOOP_PERCENT_OUTPUT_HI = 1.0;

    public static final double OPEN_LOOP_VOLTAGE_RAMP_HI = 0.3;
    public static final double OPEN_LOOP_VOLTAGE_RAMP_LO = 0.3;

    private double m_moveInput = 0.0;
    private double m_steerInput = 0.0;

    private double m_moveOutput = 0.0;
    private double m_steerOutput = 0.0;

    private boolean isHighGear = false;

    // Left Drive
    private WPI_TalonFX mLeftMaster;
    private TalonFX mLeftSlave1;
    private TalonFX mLeftSlave2;

    // Right Drive
    private WPI_TalonFX mRightMaster;
    private TalonFX mRightSlave1;
    private TalonFX mRightSlave2;

    // Gyro
    private PigeonIMU gyroPigeon;
    private double[] yprPigeon = new double[3];
    private short[] xyzPigeon = new short[3];
    private boolean isCalibrating = false;
    private double gyroYawOffsetAngleDeg = Constants.DRIVE_COMPETITION_GYRO_HOME_ANGLE_DEGREES;

    // Differential Drive
    private DifferentialDrive m_drive;
    private GameController m_driverController;

    //Path Following
    private final DifferentialDriveOdometry m_odometry;


    // Subsystem Instance
    private final static Drive INSTANCE = new Drive();

    private Drive() {
        mLeftMaster = new WPI_TalonFX(Constants.DRIVETRAIN_LEFT_MOTOR_MASTER_CAN_ID);
        mLeftSlave1 = new TalonFX(Constants.DRIVETRAIN_LEFT_MOTOR_SLAVE_1_CAN_ID);
        mLeftSlave2 = new TalonFX(Constants.DRIVETRAIN_LEFT_MOTOR_SLAVE_2_CAN_ID);

        mRightMaster = new WPI_TalonFX(Constants.DRIVETRAIN_RIGHT_MOTOR_MASTER_CAN_ID);
        mRightSlave1 = new TalonFX(Constants.DRIVETRAIN_RIGHT_MOTOR_SLAVE_1_CAN_ID);
        mRightSlave2 = new TalonFX(Constants.DRIVETRAIN_RIGHT_MOTOR_SLAVE_2_CAN_ID);

        TalonFXConfiguration configs = new TalonFXConfiguration();
        configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;

        mLeftMaster.configAllSettings(configs);
        mLeftSlave1.configAllSettings(configs);
        mLeftSlave2.configAllSettings(configs);

        mRightMaster.configAllSettings(configs);
        mRightSlave1.configAllSettings(configs);
        mRightSlave2.configAllSettings(configs);

        mLeftMaster.setInverted(TalonFXInvertType.CounterClockwise);
        mLeftSlave1.setInverted(TalonFXInvertType.FollowMaster);
        mLeftSlave2.setInverted(TalonFXInvertType.FollowMaster);

        mLeftMaster.setNeutralMode(NeutralMode.Brake);
        mLeftSlave1.setNeutralMode(NeutralMode.Brake);
        mLeftSlave2.setNeutralMode(NeutralMode.Brake);

        mLeftSlave1.follow(mLeftMaster);
        mLeftSlave2.follow(mLeftMaster);

        mRightMaster.setInverted(TalonFXInvertType.CounterClockwise);
        mRightSlave1.setInverted(TalonFXInvertType.FollowMaster);
        mRightSlave2.setInverted(TalonFXInvertType.FollowMaster);

        mRightMaster.setNeutralMode(NeutralMode.Brake);
        mRightSlave1.setNeutralMode(NeutralMode.Brake);
        mRightSlave2.setNeutralMode(NeutralMode.Brake);

        mRightSlave1.follow(mRightMaster);
        mRightSlave2.follow(mRightMaster);

        mLeftMaster.enableVoltageCompensation(true);
        mLeftMaster.configVoltageCompSaturation(12.0);
        mLeftMaster.configPeakOutputForward(+1.0f);
        mLeftMaster.configPeakOutputReverse(-1.0f);

        mRightMaster.enableVoltageCompensation(true);
        mRightMaster.configVoltageCompSaturation(12.0);
        mRightMaster.configPeakOutputForward(+1.0f);
        mRightMaster.configPeakOutputReverse(-1.0f);

        SupplyCurrentLimitConfiguration supplyCurrentConfigs = new SupplyCurrentLimitConfiguration();
        supplyCurrentConfigs.currentLimit = 60;
        supplyCurrentConfigs.enable = false;

        mLeftMaster.configSupplyCurrentLimit(supplyCurrentConfigs);
        mLeftSlave1.configSupplyCurrentLimit(supplyCurrentConfigs);
        mLeftSlave2.configSupplyCurrentLimit(supplyCurrentConfigs);

        mRightMaster.configSupplyCurrentLimit(supplyCurrentConfigs);
        mRightSlave1.configSupplyCurrentLimit(supplyCurrentConfigs);
        mRightSlave2.configSupplyCurrentLimit(supplyCurrentConfigs);

        StatorCurrentLimitConfiguration statorCurrentConfigs = new StatorCurrentLimitConfiguration();
        statorCurrentConfigs.currentLimit = 60;
        statorCurrentConfigs.triggerThresholdCurrent = 80;
        statorCurrentConfigs.triggerThresholdTime = 0.5;
        statorCurrentConfigs.enable = true;

        mLeftMaster.configStatorCurrentLimit(statorCurrentConfigs);
        mLeftSlave1.configStatorCurrentLimit(statorCurrentConfigs);
        mLeftSlave2.configStatorCurrentLimit(statorCurrentConfigs);

        mRightMaster.configStatorCurrentLimit(statorCurrentConfigs);
        mRightSlave1.configStatorCurrentLimit(statorCurrentConfigs);
        mRightSlave2.configStatorCurrentLimit(statorCurrentConfigs);

        m_drive = new DifferentialDrive(mLeftMaster, mRightMaster);
        m_drive.setSafetyEnabled(false);

        gyroPigeon = new PigeonIMU(Constants.GYRO_CAN_ID);
        gyroPigeon.configFactoryDefault();
//        gyroPigeon.setStatusFramePeriod(10, 10);

        updateOpenLoopVoltageRamp();

        m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getGyroFusedHeadingAngleDeg()));

    }

    public static Drive getInstance() {
        return INSTANCE;
    }

    //Encoder Setup
    public synchronized DriveControlMode getControlMode() {
        return driveControlMode;
    }

    public synchronized void setControlMode(DriveControlMode controlMode) {
        this.driveControlMode = controlMode;
    }

    // Encoder Setup

    //Returns left sensors velocity in ticks per 100ms
    public double getLeftVelocityNativeUnits() {
        return mLeftMaster.getSelectedSensorVelocity(0);
    }

    //Returns right sensors velocity in ticks per 100ms
    public double getRightVelocityNativeUnits() {
        return -mRightMaster.getSelectedSensorVelocity(0);
    }

    //Returns left sensors position in ticks
    public double getLeftSensorPosition(){
        return mLeftMaster.getSelectedSensorPosition(0);
    }

    //Returns right sensors position in ticks
    public double getRightSensorPosition(){
        return -mRightMaster.getSelectedSensorPosition(0);
    }

    //Takes that times the wheel has rotated * by the circumference of the wheel to get its distance traveled in inches
    public static double rotationsToInches(double rotations) {
        return rotations * (Constants.kWheelDiameterInches * Math.PI);
    }

    //Takes inches and converts it to meters using units class
    public static double inchesToMeters(double inches){
        return Units.inchesToMeters(inches);
    }

    //Takes the sensor velocity of an encoder * by 10 to get ticks per second / the encoder PPR to get encoder rotations
    //per second and then uses the rotations to inches functions to get inches per second
    private static double ticksPer100msToInchesPerSec(double ticks_100ms) {
        return rotationsToInches(ticks_100ms * 10.0 / Constants.ENCODER_TICKS_PER_MOTOR_REVOLUTION);
    }

    //Returns left inches per second using the sensor velocity and the ticksToInches conversion method
    public double getLeftInchesPerSecond(){
        return ticksPer100msToInchesPerSec(getLeftVelocityNativeUnits()) / DRIVE_OUTPUT_TO_ENCODER_RATIO;
    }

    //Returns right inches per second using the sensor velocity and the ticksToInches conversion method
    public double getRightInchesPerSecond(){
        return ticksPer100msToInchesPerSec(getRightVelocityNativeUnits()) / DRIVE_OUTPUT_TO_ENCODER_RATIO;
    }

    //Returns left meters per second using inchesPerSecond calculation and inchesToMeters method
    public double getLeftMetersPerSecond(){
        return inchesToMeters(getLeftInchesPerSecond());
    }

    //Returns right meters per second using inchesPerSecond calculation and inchesToMeters method
    public double getRightMetersPerSecond(){
        return inchesToMeters(getRightInchesPerSecond());
    }

    //Sensors positions in ticks / Pulses per Revolution of the Encoder = Encoder Rotations (If ratio is 1:1)
    public double getLeftEncoderRotations() {
        return getLeftSensorPosition() / Constants.ENCODER_TICKS_PER_MOTOR_REVOLUTION;
    }

    public double getRightEncoderRotations() {
        return getRightSensorPosition() / Constants.ENCODER_TICKS_PER_MOTOR_REVOLUTION;
    }

    //Wheel Rotations = Encoder Rotations (If ratio is 1:1)
    public double getLeftWheelRotations() {
        return getLeftEncoderRotations() / DRIVE_OUTPUT_TO_ENCODER_RATIO;
    }

    public double getRightWheelRotations() {
        return getRightEncoderRotations() / DRIVE_OUTPUT_TO_ENCODER_RATIO;
    }

    //Returns left distance traveled in inches by taking wheel rotations and converting it to inches
    public double getLeftWheelDistanceInches() {
        return rotationsToInches(getLeftWheelRotations());
    }

    //Returns right distance traveled in inches by taking wheel rotations and converting it to inches
    public double getRightWheelDistanceInches() {
        return rotationsToInches(getRightWheelRotations());
    }

    //Returns left distance traveled in meters using calculated inches distances and inchesToMeters conversion
    public double getLeftWheelDistanceMeters() {
        return inchesToMeters(getLeftWheelDistanceInches());
    }

    //Returns right distance traveled in meters using calculated inches distances and inchesToMeters conversion
    public double getRightWheelDistanceMeters(){
        return inchesToMeters(getRightWheelDistanceInches());
    }

    public double getRobotDistanceFromGoal(){ return Units.metersToInches(getPose().getTranslation().getDistance(Constants.GOAL_ORIGIN)); }

    public synchronized void resetEncoders() {
        mLeftMaster.setSelectedSensorPosition(0);
        mRightMaster.setSelectedSensorPosition(0);
    }

    // Gyro Set Up
    public void calibrateGyro() {
        gyroPigeon.enterCalibrationMode(PigeonIMU.CalibrationMode.Temperature);
    }

    public void endGyroCalibration() {
        if (isCalibrating == true) {
            isCalibrating = false;
        }
    }

    public void setGyroYawOffset(double offsetDeg) {
        gyroYawOffsetAngleDeg = offsetDeg;
    }

    public synchronized double getGyroYawAngleDeg() {
        gyroPigeon.getYawPitchRoll(yprPigeon);
        return yprPigeon[0] + gyroYawOffsetAngleDeg;
    }

    public synchronized double getGyroFusedHeadingAngleDeg() {
        return gyroPigeon.getFusedHeading() + gyroYawOffsetAngleDeg;
    }

    public synchronized double getGyroPitchAngle() {
        gyroPigeon.getYawPitchRoll(yprPigeon);
        return yprPigeon[2];
    }

    public synchronized void resetGyroYawAngle() {
        gyroPigeon.setYaw(0);
        gyroPigeon.setFusedHeading(0);
    }

    public synchronized void resetGyroYawAngle(double homeAngle) {
        resetGyroYawAngle();
        setGyroYawOffset(homeAngle);
    }

    private void updateOpenLoopVoltageRamp() {
        setOpenLoopVoltageRamp(isHighGear ? OPEN_LOOP_VOLTAGE_RAMP_HI : OPEN_LOOP_VOLTAGE_RAMP_LO);
    }

    private void setOpenLoopVoltageRamp(double timeTo12VSec) {
        mLeftMaster.configOpenloopRamp(timeTo12VSec);
        mRightMaster.configOpenloopRamp(timeTo12VSec);
    }

    public synchronized void driveWithJoystick() {
        if (m_drive == null) {
            return;
        }

        boolean isHighGearPrevious = isHighGear;
 //       isHighGear = m_driverController.getRightBumper().get();
        isHighGear = m_driverController.getRightTrigger().get();
        if (isHighGearPrevious != isHighGear) {
            updateOpenLoopVoltageRamp();
        }

        double shiftScaleFactor = OPEN_LOOP_PERCENT_OUTPUT_LO;
        if (isHighGear == true) {
            shiftScaleFactor = OPEN_LOOP_PERCENT_OUTPUT_HI;
        }

        m_moveInput = -m_driverController.getLeftYAxis();
        m_steerInput = m_driverController.getRightXAxis();

        m_moveOutput = adjustForSensitivity(MOVE_SCALE * shiftScaleFactor, MOVE_TRIM, m_moveInput, MOVE_NON_LINEAR, MOVE_NON_LINEARITY);
        m_steerOutput = adjustForSensitivity(STEER_SCALE, STEER_TRIM, m_steerInput, STEER_NON_LINEAR, STEER_NON_LINEARITY);

        m_drive.arcadeDrive(m_moveOutput, m_steerOutput);
    }

    public double adjustForSensitivity(double scale, double trim, double steer, int nonLinearFactor,
                                       double wheelNonLinearity) {
        if (inDeadZone(steer))
            return 0;

        steer += trim;
        steer *= scale;
        steer = limitValue(steer);

        int iterations = Math.abs(nonLinearFactor);
        for (int i = 0; i < iterations; i++) {
            if (nonLinearFactor > 0) {
                steer = nonlinearStickCalcPositive(steer, wheelNonLinearity);
            } else {
                steer = nonlinearStickCalcNegative(steer, wheelNonLinearity);
            }
        }
        return steer;
    }

    private boolean inDeadZone(double input) {
        boolean inDeadZone;
        if (Math.abs(input) < STICK_DEADBAND) {
            inDeadZone = true;
        } else {
            inDeadZone = false;
        }
        return inDeadZone;
    }

    private double limitValue(double value) {
        if (value > 1.0) {
            value = 1.0;
        } else if (value < -1.0) {
            value = -1.0;
        }
        return value;
    }

    private double nonlinearStickCalcPositive(double steer, double steerNonLinearity) {
        return Math.sin(Math.PI / 2.0 * steerNonLinearity * steer) / Math.sin(Math.PI / 2.0 * steerNonLinearity);
    }

    private double nonlinearStickCalcNegative(double steer, double steerNonLinearity) {
        return Math.asin(steerNonLinearity * steer) / Math.asin(steerNonLinearity);
    }
    public void setDriverController(GameController driverController) {
        m_driverController = driverController;
    }

    //Path Following
    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    /**
     * Returns the current wheel speeds of the robot.
     *
     * @return The current wheel speeds.
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getLeftMetersPerSecond(), getRightMetersPerSecond());
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        resetGyroYawAngle(Constants.DRIVE_COMPETITION_GYRO_HOME_ANGLE_DEGREES);
        m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getGyroFusedHeadingAngleDeg()));
    }

    /**
     * Controls the left and right sides of the drive directly with voltages.
     *
     * @param leftVolts  the commanded left output
     * @param rightVolts the commanded right output
     */
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        mLeftMaster.setVoltage(leftVolts);
        mRightMaster.setVoltage(-rightVolts);
        m_drive.feed();
    }

    public void periodic() {
        synchronized (Drive.this){
            DriveControlMode currentControlMode = getControlMode();
            switch (currentControlMode){
                case JOYSTICK:
                    driveWithJoystick();
                    break;
                case PATH_FOLLOWING:
                    break;
                default:
                    System.out.println("Unknown drive control mode: " + currentControlMode);
            }
            m_odometry.update(Rotation2d.fromDegrees(getGyroFusedHeadingAngleDeg()),
                    getLeftWheelDistanceMeters(),getRightWheelDistanceMeters());
        }


        SmartDashboard.putNumber("Left Distance Inches: ", getLeftWheelDistanceInches());
       SmartDashboard.putNumber("Right Distance Inches: ", getRightWheelDistanceInches());

       SmartDashboard.putNumber("Left Distance Meters: ", getLeftWheelDistanceMeters());
        SmartDashboard.putNumber("Right Distance Meters: ", getRightWheelDistanceMeters());

        SmartDashboard.putNumber("Heading: ", getGyroFusedHeadingAngleDeg());
        SmartDashboard.putNumber("X Pose", Units.metersToInches(getPose().getTranslation().getX()));
        SmartDashboard.putNumber("Y Pose", Units.metersToInches(getPose().getTranslation().getY()));
        SmartDashboard.putNumber("Robot to Goal Angle: ", Math.toDegrees(Math.acos(Units.metersToInches(getPose().getTranslation().getX())/
                Units.metersToInches(getPose().getTranslation().getDistance(Constants.GOAL_ORIGIN)))));
        SmartDashboard.putNumber("Robot to Goal Distance", Units.metersToInches(getPose().getTranslation().getDistance(Constants.GOAL_ORIGIN)));
        SmartDashboard.putNumber("Rotation", getPose().getRotation().getDegrees());
    }
}


