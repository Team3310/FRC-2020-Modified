package frc.robot.auto;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.Constants;

import java.util.List;

public class TrajectoryGenerator {
    private static final TrajectoryGenerator instance = new TrajectoryGenerator();

    public static TrajectoryGenerator getInstance() {
        return instance;
    }

    // Create a voltage constraint to ensure we don't accelerate too fast
    DifferentialDriveVoltageConstraint autoVoltageConstraint =
            new DifferentialDriveVoltageConstraint(
                    new SimpleMotorFeedforward(Constants.ksVolts,
                            Constants.kvVoltSecondsPerMeter,
                            Constants.kaVoltSecondsSquaredPerMeter),
                    Constants.kDriveKinematics,
                    10);

    // Create config for trajectory
    TrajectoryConfig forwardConfig =
            new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond,
                    Constants.kMaxAccelerationMetersPerSecondSquared)
                    // Add kinematics to ensure max speed is actually obeyed
                    .setKinematics(Constants.kDriveKinematics)
                    // Apply the voltage constraint
                    .addConstraint(autoVoltageConstraint);

    TrajectoryConfig forwardMediumConfig =
            new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond * 1.2,
                    Constants.kMaxAccelerationMetersPerSecondSquared)
                    // Add kinematics to ensure max speed is actually obeyed
                    .setKinematics(Constants.kDriveKinematics)
                    // Apply the voltage constraint
                    .addConstraint(autoVoltageConstraint);

    TrajectoryConfig forwardConfigSlow =
            new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond * 0.5,
                    Constants.kMaxAccelerationMetersPerSecondSquared)
                    // Add kinematics to ensure max speed is actually obeyed
                    .setKinematics(Constants.kDriveKinematics)
                    // Apply the voltage constraint
                    .addConstraint(autoVoltageConstraint);

    TrajectoryConfig forwardFastConfig =
            new TrajectoryConfig(Constants.kMaxFastSpeedMetersPerSecond,
                    Constants.kMaxAccelerationMetersPerSecondSquared)
                    // Add kinematics to ensure max speed is actually obeyed
                    .setKinematics(Constants.kDriveKinematics)
                    // Apply the voltage constraint
                    .addConstraint(autoVoltageConstraint);

    TrajectoryConfig reverseConfig =
            new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond,
                    Constants.kMaxAccelerationMetersPerSecondSquared)
                    // Add kinematics to ensure max speed is actually obeyed
                    .setKinematics(Constants.kDriveKinematics)
                    // Apply the voltage constraint
                    .addConstraint(autoVoltageConstraint)
                    .setReversed(true);

    TrajectoryConfig reverseConfigSlow =
            new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond * 0.5,
                    Constants.kMaxAccelerationMetersPerSecondSquared)
                    // Add kinematics to ensure max speed is actually obeyed
                    .setKinematics(Constants.kDriveKinematics)
                    // Apply the voltage constraint
                    .addConstraint(autoVoltageConstraint);

    TrajectoryConfig reverseFastConfig =
            new TrajectoryConfig(Constants.kMaxFastSpeedMetersPerSecond,
                    Constants.kMaxFastAccelerationMetersPerSecondSquared)
                    // Add kinematics to ensure max speed is actually obeyed
                    .setKinematics(Constants.kDriveKinematics)
                    // Apply the voltage constraint
                    .addConstraint(autoVoltageConstraint)
                    .setReversed(true);


    public Trajectory getDriveStraight(){
            return edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                    new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0),
                            new Rotation2d(Units.radiansToDegrees(0))),
                    List.of(
                            new Translation2d(Units.inchesToMeters(60), Units.inchesToMeters(0))
                    ),
                    new Pose2d(Units.inchesToMeters(120), Units.inchesToMeters(0),
                            new Rotation2d(Units.radiansToDegrees(0))),
                    // Pass config
                    forwardConfig
            );
        }

        public Trajectory getDriveStraightReversed(){
            return edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                    new Pose2d(Units.inchesToMeters(120), Units.inchesToMeters(0),
                            new Rotation2d(Units.radiansToDegrees(0))),
                    List.of(
                            new Translation2d(Units.inchesToMeters(60), Units.inchesToMeters(0))
                    ),
                    new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0),
                            new Rotation2d(Units.radiansToDegrees(0))),
                    // Pass config
                    reverseConfig
            );
        }

        //Start 8 Ball Trench Auto
        public Trajectory getCenterStartToEndOfTrench() {
        Trajectory centerStartToEndOfTrench;
        centerStartToEndOfTrench = edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0), new Rotation2d(0)),
                List.of(
                        new Translation2d(Units.inchesToMeters(60), Units.inchesToMeters(40)),
                        new Translation2d(Units.inchesToMeters(80), Units.inchesToMeters(60)),
                        new Translation2d(Units.inchesToMeters(120), Units.inchesToMeters(63))
                ),
                new Pose2d(Units.inchesToMeters(200), Units.inchesToMeters(65.66), new Rotation2d(0)),
                // Pass config
                forwardConfig
        );
        return centerStartToEndOfTrench;

    }
    public Trajectory getToStartTrench() {
        Trajectory ToStartTrench;
        ToStartTrench = edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0), new Rotation2d(0)),
                List.of(
                        new Translation2d(Units.inchesToMeters(60), Units.inchesToMeters(0))

                ),
                new Pose2d(Units.inchesToMeters(70), Units.inchesToMeters(0), new Rotation2d(0)),
                // Pass config
                forwardConfig
        );
        return ToStartTrench;
    }
    public Trajectory getToTrenchToTrench() {
        Trajectory ToTrenchToTrench;
        ToTrenchToTrench = edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                new Pose2d(Units.inchesToMeters(70), Units.inchesToMeters(0), new Rotation2d(0)),
                List.of(
                        new Translation2d(Units.inchesToMeters(150), Units.inchesToMeters(0))

                ),
                new Pose2d(Units.inchesToMeters(200), Units.inchesToMeters(0), new Rotation2d(0)),
                // Pass config
                forwardConfig
        );
        return ToTrenchToTrench;
    }
    public Trajectory getPanelToTrench() {
        Trajectory PanelToTrench;
        PanelToTrench = edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                new Pose2d(Units.inchesToMeters(200), Units.inchesToMeters(0), new Rotation2d(0)),
                List.of(
                        new Translation2d(Units.inchesToMeters(150), Units.inchesToMeters(0))

                ),
                new Pose2d(Units.inchesToMeters(70), Units.inchesToMeters(0), new Rotation2d(0)),
                // Pass config
                reverseConfig
        );
        return PanelToTrench;
    }



    public Trajectory getEndOfTrenchToStartOfTrench() {
            Trajectory endOfTrenchToStartOfTrench;
            endOfTrenchToStartOfTrench = edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                    new Pose2d(Units.inchesToMeters(200), Units.inchesToMeters(65.66), new Rotation2d(0)),
                    List.of(
                            new Translation2d(Units.inchesToMeters(120), Units.inchesToMeters(64))
                    ),
                    new Pose2d(Units.inchesToMeters(70), Units.inchesToMeters(50), new Rotation2d(0)),
                    // Pass config
                    reverseConfig
            );
            return endOfTrenchToStartOfTrench;
        }
        //End 8 Ball Trench Auto

        //Start 5 Ball Steal Auto
        public Trajectory getStealStartToStealBall() {
            Trajectory stealStartToStealSpot;
            stealStartToStealSpot = edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                    new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0), new Rotation2d(0)),
                    List.of(
                            new Translation2d(Units.inchesToMeters(60), Units.inchesToMeters(0))
                    ),
                    new Pose2d(Units.inchesToMeters(130), Units.inchesToMeters(0), new Rotation2d(0)),
                    // Pass config
                    forwardConfig
            );
            return stealStartToStealSpot;
        }

        public Trajectory getStealStartToStealBallV2() {
            Trajectory stealStartToStealSpot;
            stealStartToStealSpot = edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                new Pose2d(Units.inchesToMeters(147), Units.inchesToMeters(-245), new Rotation2d(0)),
                List.of(
                        new Translation2d(Units.inchesToMeters(201), Units.inchesToMeters(-264))
                ),
                new Pose2d(Units.inchesToMeters(270), Units.inchesToMeters(-302), Rotation2d.fromDegrees(-45.0)),
                // Pass config
                forwardFastConfig
            );
            return stealStartToStealSpot;
        }

        public Trajectory getStealStartToStealBallV3() {
            Trajectory stealStartToStealSpot;
            stealStartToStealSpot = edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                    new Pose2d(Units.inchesToMeters(126), Units.inchesToMeters(-245), new Rotation2d(0)),
                    List.of(
                            new Translation2d(Units.inchesToMeters(201), Units.inchesToMeters(-264))
                    ),
                    new Pose2d(Units.inchesToMeters(267), Units.inchesToMeters(-304), Rotation2d.fromDegrees(-45.0)),
                    // Pass config
                    forwardConfigSlow
            );
            return stealStartToStealSpot;
        }

        public Trajectory getStealBallToCenterShot() {
            Trajectory stealSpotToCenterShot;
            stealSpotToCenterShot = edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                    new Pose2d(Units.inchesToMeters(130), Units.inchesToMeters(0), new Rotation2d(0)),
                    List.of(
                            new Translation2d(Units.inchesToMeters(60), Units.inchesToMeters(60))
                    ),
                    new Pose2d(Units.inchesToMeters(-20), Units.inchesToMeters(190), new Rotation2d(0)),
                    // Pass config
                    reverseConfig
            );
            return stealSpotToCenterShot;
        }

        public Trajectory getStealBallToCenterShotV2() {
            Trajectory stealSpotToCenterShot;
            stealSpotToCenterShot = edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                    new Pose2d(Units.inchesToMeters(270), Units.inchesToMeters(-302), Rotation2d.fromDegrees(-45)),
                    List.of(
                            new Translation2d(Units.inchesToMeters(228), Units.inchesToMeters(-254)),
                            new Translation2d(Units.inchesToMeters(196), Units.inchesToMeters(-192))
    //                        new Translation2d(Units.inchesToMeters(185), Units.inchesToMeters(-171))
                    ),
                    new Pose2d(Units.inchesToMeters(185), Units.inchesToMeters(-120), Rotation2d.fromDegrees(-71)),
                    // Pass config
                    reverseFastConfig
            );
            return stealSpotToCenterShot;
        }

        // Start 8 Ball Steal Auto
        public Trajectory getStealFarSideRendezvousPoint2Balls() {
            Trajectory stealFarSideRendezvousPoint2Balls = edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
               new Pose2d(Units.inchesToMeters(185), Units.inchesToMeters(-120), Rotation2d.fromDegrees(-71)),
                List.of(
                        new Translation2d(Units.inchesToMeters(199), Units.inchesToMeters(-149))

                ),
                new Pose2d(Units.inchesToMeters(250), Units.inchesToMeters(-164), Rotation2d.fromDegrees(28)), // 10
                // Pass config
                forwardConfig
            );

            return stealFarSideRendezvousPoint2Balls;
        }

        public Trajectory getStealFarSideRendezvousPoint2BallsReverse() {
            Trajectory stealFarSideRendezvousPoint2Balls = edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                    new Pose2d(Units.inchesToMeters(250), Units.inchesToMeters(-164), Rotation2d.fromDegrees(28)),
                    List.of(
                            new Translation2d(Units.inchesToMeters(199), Units.inchesToMeters(-149))

                    ),
                    new Pose2d(Units.inchesToMeters(185), Units.inchesToMeters(-120), Rotation2d.fromDegrees(-71)), // 10
                    // Pass config
                    reverseConfig
            );

            return stealFarSideRendezvousPoint2Balls;
        }

        public Trajectory getStealFarSideRendezvousPointRetreat() {
        Trajectory stealFarSideRendezvousPointRetreat = edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                new Pose2d(Units.inchesToMeters(237), Units.inchesToMeters(-169), new Rotation2d(22)),
                List.of(
                        new Translation2d(Units.inchesToMeters(65), Units.inchesToMeters(55))
                ),
                new Pose2d(Units.inchesToMeters(24), Units.inchesToMeters(34), new Rotation2d(22)),
                // Pass config
                reverseConfig
             );
            return stealFarSideRendezvousPointRetreat;
        }

        public Trajectory getStealFarSideRendezvousPointThirdBall() {
        Trajectory stealFarSideRendezvousPointThirdBall = edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                new Pose2d(Units.inchesToMeters(24), Units.inchesToMeters(34), new Rotation2d(42)),
                List.of(
                        new Translation2d(Units.inchesToMeters(57), Units.inchesToMeters(64))
                ),
                new Pose2d(Units.inchesToMeters(91), Units.inchesToMeters(92), new Rotation2d(42)),
                // Pass config
                forwardConfigSlow
        );
        return stealFarSideRendezvousPointThirdBall;
    }

        public Trajectory getStealFarSideRendezvousPointThreeBallShot() {
        Trajectory stealFarSideRendezvousPointThreeBallShot = edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                new Pose2d(Units.inchesToMeters(91), Units.inchesToMeters(92), new Rotation2d(-17)),
                List.of(
                        new Translation2d(Units.inchesToMeters(62), Units.inchesToMeters(102))
                ),
                new Pose2d(Units.inchesToMeters(21), Units.inchesToMeters(115), new Rotation2d(-6)),
                // Pass config
                reverseConfig
        );
        return stealFarSideRendezvousPointThreeBallShot;
        }
        // End 8 Ball Steal Auto

        //Start 10 Ball Rendezvous/Trench Auto
        public Trajectory getCenterStartToRendezvous2ball() {
            Trajectory centerStartToRendezvous2Ball;
            centerStartToRendezvous2Ball = edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                    new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0), new Rotation2d(0)),
                    List.of(
                            new Translation2d(Units.inchesToMeters(80), Units.inchesToMeters(40))
                    ),
                    new Pose2d(Units.inchesToMeters(155), Units.inchesToMeters(-10), new Rotation2d(Units.degreesToRadians(-60))),
                    // Pass config
                    forwardConfig
            );
            return centerStartToRendezvous2Ball;
        }

        public Trajectory getRendezvous2BallToStartOfTrench() {
            Trajectory rendezvous2BallToStartOfTrench;
            rendezvous2BallToStartOfTrench = edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                    new Pose2d(Units.inchesToMeters(155), Units.inchesToMeters(-10), new Rotation2d(Units.degreesToRadians(-60))),
                    List.of(
                            new Translation2d(Units.inchesToMeters(100), Units.inchesToMeters(30))
                    ),
                    new Pose2d(Units.inchesToMeters(70), Units.inchesToMeters(65), new Rotation2d(Units.degreesToRadians(0))),
                    // Pass config
                    reverseConfig
            );
            return rendezvous2BallToStartOfTrench;
        }

        public Trajectory getStartOfTrenchToEndOfTrench() {
            Trajectory startOfTrenchToEndOfTrench;
            startOfTrenchToEndOfTrench = edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                    new Pose2d(Units.inchesToMeters(70), Units.inchesToMeters(65), new Rotation2d(0)),
                    List.of(
                            new Translation2d(Units.inchesToMeters(100), Units.inchesToMeters(65.66))
                    ),
                    new Pose2d(Units.inchesToMeters(300), Units.inchesToMeters(65.66), new Rotation2d(Units.degreesToRadians(0))),
                    // Pass config
                    forwardConfig
            );
            return startOfTrenchToEndOfTrench;
        }

        //Start 3 Ball Safe Auto
        public Trajectory getLeftStartToSafe(){
            Trajectory leftStartToSafe;
            leftStartToSafe = edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                    new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0), new Rotation2d(Units.degreesToRadians(0))),
                    List.of(
                            new Translation2d(Units.inchesToMeters(-40), Units.inchesToMeters(0))
                    ),
                    new Pose2d(Units.inchesToMeters(-60), Units.inchesToMeters(0), new Rotation2d(Units.degreesToRadians(0))),
                    // Pass config
                    reverseConfig
            );
            return leftStartToSafe;
        }

    public Trajectory getLeftStartToSafeForward(){
        Trajectory leftStartToSafe;
        leftStartToSafe = edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0), new Rotation2d(Units.degreesToRadians(0))),
                List.of(
                        new Translation2d(Units.inchesToMeters(40), Units.inchesToMeters(0))
                ),
                new Pose2d(Units.inchesToMeters(60), Units.inchesToMeters(0), new Rotation2d(Units.degreesToRadians(0))),
                // Pass config
                forwardConfig
        );
        return leftStartToSafe;
    }
        //End 3 Ball Safe Auto

    // Test
    public Trajectory getTestAuton(){
        Trajectory testAuton;
        testAuton = edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                new Pose2d(Units.inchesToMeters(76), Units.inchesToMeters(-155), new Rotation2d(Units.degreesToRadians(0))),
                List.of(
                        new Translation2d(Units.inchesToMeters(100), Units.inchesToMeters(-155))
                ),
                new Pose2d(Units.inchesToMeters(120), Units.inchesToMeters(-155), new Rotation2d(Units.degreesToRadians(0))),
                // Pass config
                forwardConfig
        );
        return testAuton;
    }
    public Trajectory getFirstTwoBalls() {
        Trajectory FirstTwoBalls;
        FirstTwoBalls = edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                new Pose2d(Units.inchesToMeters(136), Units.inchesToMeters(-60), new Rotation2d(Units.degreesToRadians(0))),
                List.of(
                        new Translation2d(Units.inchesToMeters(213.098), Units.inchesToMeters(-77.632))
                ),
                new Pose2d(Units.inchesToMeters(245.661), Units.inchesToMeters(-129.337), new Rotation2d(Units.degreesToRadians(-60))),
                forwardMediumConfig
        );
        return FirstTwoBalls;

    }
    public Trajectory getFirstTwoBallsReversed() {
        Trajectory FirstTwoBallsReversed;
        FirstTwoBallsReversed = edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                new Pose2d(Units.inchesToMeters(245.661), Units.inchesToMeters(-129.337), new Rotation2d(Units.degreesToRadians(-60))),
                List.of(
                        new Translation2d(Units.inchesToMeters(213.098), Units.inchesToMeters(-77.632))
                ),
                new Pose2d(Units.inchesToMeters(136), Units.inchesToMeters(-60), new Rotation2d(Units.degreesToRadians(0))),
                reverseFastConfig
        );
        return FirstTwoBallsReversed;
    }
    public Trajectory getTrench3Ball() {
        Trajectory Trench3Ball;
        Trench3Ball = edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                new Pose2d(Units.inchesToMeters(136), Units.inchesToMeters(-60), new Rotation2d(Units.degreesToRadians(0))),
                List.of(
                        new Translation2d(Units.inchesToMeters(181.547), Units.inchesToMeters(-39.622)),
                        new Translation2d(Units.inchesToMeters(212.104), Units.inchesToMeters(-30.927))

                ),

                new Pose2d(Units.inchesToMeters(316.694), Units.inchesToMeters(-28.691), new Rotation2d(Units.degreesToRadians(0))),
                forwardConfig
        );
        return Trench3Ball;

    }
    public Trajectory getToMediumFromPanel() {
        Trajectory ToMediumFromPanel;
        ToMediumFromPanel = edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                new Pose2d(Units.inchesToMeters(316.694), Units.inchesToMeters(-28.691), new Rotation2d(Units.degreesToRadians(0))),
                List.of(
                        new Translation2d(Units.inchesToMeters(290.000), Units.inchesToMeters(-28.691))

                ),

                new Pose2d(Units.inchesToMeters(199.755), Units.inchesToMeters(-28.691), new Rotation2d(Units.degreesToRadians(0))),
                reverseFastConfig
        );
        return ToMediumFromPanel;

    }










}




//}
