package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Turret;


public class TurretSetLagAngle extends ExtraTimeoutCommand {
    private final Drive drive;
    private final Turret turret;
    public double rCurrPoseX;
    public double rCurrPoseY;
    public double rDistToGoal;
    public double rGoalAngleDeg;
    public double rGoalAngleDegOffset;
    public double offsetY;
    public double turrOffsetAngle;


    public TurretSetLagAngle(Drive robot, Turret turret){
        this.drive = robot;
        this.turret = turret;

    }
    @Override
    public void initialize() {

        rCurrPoseX = Units.metersToInches(drive.getPose().getTranslation().getX());
        rCurrPoseY = Units.metersToInches(drive.getPose().getTranslation().getY());
        rDistToGoal = Units.metersToInches(drive.getPose().getTranslation().getDistance(Constants.GOAL_ORIGIN));
        rGoalAngleDeg = Math.toDegrees(Math.acos(Units.metersToInches(drive.getPose().getTranslation().getX())/
                Units.metersToInches(drive.getPose().getTranslation().getDistance(Constants.GOAL_ORIGIN))));
        offsetY = drive.getLeftMetersPerSecond() * Constants.FLIGHT_TIME_OF_BALL;
        turrOffsetAngle = Math.toDegrees(Math.atan(offsetY/rDistToGoal));
        rGoalAngleDegOffset = rGoalAngleDeg + turrOffsetAngle;

     turret.setTurretMotionMagicPositionAbsolute(-180 +rGoalAngleDeg + (-1* drive.getGyroFusedHeadingAngleDeg()));
        if (rCurrPoseY < -95){
            turret.setTurretMotionMagicPositionAbsolute(-180 - Math.toDegrees(Math.acos(Units.metersToInches(drive.getPose().getTranslation().getX())/
                    Units.metersToInches(drive.getPose().getTranslation().getDistance(Constants.GOAL_ORIGIN)))) );
        }



//        new TurretSetToGyroAngle(turret, Math.toDegrees(Math.acos(Units.metersToInches(drive.getPose().getTranslation().getX())/
//                Units.metersToInches(drive.getPose().getTranslation().getDistance(Constants.GOAL_ORIGIN))))); // Turret Angle w/out Offset
////        new TurretSetToGyroAngle(turret, rGoalAngleDegOffset); // Turret Angle with Offset


    }
    @Override
    public boolean isFinished() {
        return true;
    }
}