///*----------------------------------------------------------------------------*/
///* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
///* Open Source Software - may be modified and shared by FRC teams. The code   */
///* must be accompanied by the FIRST BSD license file in the root directory of */
///* the project.                                                               */
///*----------------------------------------------------------------------------*/
//
//package frc.robot;
//
//import edu.wpi.first.wpilibj.TimedRobot;
//import edu.wpi.first.wpilibj.geometry.Pose2d;
//import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.CommandScheduler;
//import frc.robot.subsystems.*;
//
///**
// * The VM is configured to automatically run this class, and to call the
// * methods corresponding to each mode, as described in the TimedRobot
// * documentation. If you change the name of this class or the package after
// * creating this project, you must also update the build.gradle file in the
// * project.
// */
//public class Robot extends TimedRobot
//{
//    private Command autonomousCommand;
//
//    private RobotContainer robotContainer;
//    private static final Drive drive = Drive.getInstance();
//    private static final Turret turret = Turret.getInstance();
//    private static final Magazine magazine = Magazine.getInstance();
//    private static final Shooter shooter = Shooter.getInstance();
//
//    /**
//     * This method is run when the robot is first started up and should be used for any
//     * initialization code.
//     */
//    @Override
//    public void robotInit()
//    {
//        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
//        // autonomous chooser on the dashboard.
//        robotContainer = new RobotContainer();
//    }
//
//    /**
//     * This method is called every robot packet, no matter the mode. Use this for items like
//     * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
//     *
//     * <p>This runs after the mode specific periodic functions, but before
//     * LiveWindow and SmartDashboard integrated updating.
//     */
//    @Override
//    public void robotPeriodic()
//    {
//        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
//        // commands, running already-scheduled commands, removing finished or interrupted commands,
//        // and running subsystem periodic() methods.  This must be called from the robot's periodic
//        // block in order for anything in the Command-based framework to work.
//        CommandScheduler.getInstance().run();
//    }
//
//    /**
//     * This method is called once each time the robot enters Disabled mode.
//     */
//    @Override
//    public void disabledInit()
//    {
//        AirCompressor.getInstance().turnCompressorOff();
//    }
//
//    @Override
//    public void disabledPeriodic()
//    {
//    }
//
//    /**
//     * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
//     */
//    @Override
//    public void autonomousInit()
//    {
//        AirCompressor.getInstance().turnCompressorOff();
//        Limelight.getInstance().setLedMode(Limelight.LightMode.OFF);
//        turret.resetHomePosition(Constants.TURRET_COMPETITION_HOME_POSITION_DEGREES);
//        magazine.resetHomePosition();
//        shooter.resetHoodHomePosition();
//        drive.resetGyroYawAngle(Constants.DRIVE_COMPETITION_GYRO_HOME_ANGLE_DEGREES);
//        drive.setControlMode(Drive.DriveControlMode.PATH_FOLLOWING);
//        drive.resetOdometry(new Pose2d());
//
//        // schedule the autonomous command (example)
//        autonomousCommand = robotContainer.getAutonomousCommand();
//        if (autonomousCommand != null)
//        {
//            autonomousCommand.schedule();
//        }
//    }
//
//    /**
//     * This method is called periodically during autonomous.
//     */
//    @Override
//    public void autonomousPeriodic()
//    {
//    }
//
//    @Override
//    public void teleopInit()
//    {
//       drive.setControlMode(Drive.DriveControlMode.JOYSTICK);
//
//        // This makes sure that the autonomous stops running when
//        // teleop starts running. If you want the autonomous to
//        // continue until interrupted by another command, remove
//        // this line or comment it out.
//        if (autonomousCommand != null)
//        {
//            autonomousCommand.cancel();
//        }
//    }
//
//    /**
//     * This method is called periodically during operator control.
//     */
//    @Override
//    public void teleopPeriodic()
//    {
//    }
//
//    @Override
//    public void testInit()
//    {
//        // Cancels all running commands at the start of test mode.
//        CommandScheduler.getInstance().cancelAll();
//    }
//
//    /**
//     * This method is called periodically during test mode.
//     */
//    @Override
//    public void testPeriodic()
//    {
//    }
//}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.auto.routines.*;
import frc.robot.subsystems.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * methods corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot
{
    private Command m_autonomousCommand;
    private SendableChooser<Command> autonTaskChooser;

    private RobotContainer robotContainer;
    private static final Drive drive = Drive.getInstance();
    private static final Turret turret = Turret.getInstance();
    private static final Magazine magazine = Magazine.getInstance();
    private static final Shooter shooter = Shooter.getInstance();

    /**
     * This method is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit()
    {
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        robotContainer = new RobotContainer();
        drive.resetOdometry(new Pose2d());

        autonTaskChooser = new SendableChooser<>();

        autonTaskChooser.setDefaultOption("Do Nothing", new AutoDoNothing());
        autonTaskChooser.addOption("Test 8 Feet Auto", new AutoTest8Feet());

        autonTaskChooser.addOption("Trench 8 Ball Auto", new AutoTrench8Ball());
        autonTaskChooser.addOption("Trench Steal 5 Ball Auto", new AutoTrenchSteal());
        autonTaskChooser.addOption("Trench Steal 5 Ball Auto V2", new AutoTrenchStealV2());
        autonTaskChooser.addOption("Rendezvous/Trench 10 Ball Auto", new AutoRendezvousTrench10Ball());
        autonTaskChooser.addOption("Safe 3 Ball Auto", new AutoSafe());
        autonTaskChooser.addOption("Safe 3 Ball Auto Forward", new AutoSafeForward());
        autonTaskChooser.addOption("Trench 6 Ball Auto", new AutoTrench6Ball());
        autonTaskChooser.addOption("Safe 6 Ball Auto", new AutoSafe6Ball());
        autonTaskChooser.addOption("Trench Steal 7 Ball Auto", new AutoTrenchSteal7Ball());
        autonTaskChooser.addOption("Trench 8 Ball V2 Auto", new AutoTrench8BallV2());



//        autonTaskChooser.addOption("Test", new AutoTest());

        SmartDashboard.putData("Autonomous", autonTaskChooser);

       // Limelight.getInstance().setLedMode(Limelight.LightMode.OFF);
    }

    /**
     * This method is called every robot packet, no matter the mode. Use this for items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before
     * LiveWindow and SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic()
    {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
    }

    /**
     * This method is called once each time the robot enters Disabled mode.
     */
    @Override
    public void disabledInit()
    {
        Limelight.getInstance().setLedMode(Limelight.LightMode.OFF);
        Limelight.getInstance().setPipeline(Constants.LIMELIGHT_AUTO_PIPELINE);
    }

    @Override
    public void disabledPeriodic()
    {
    }

    /**
     * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        Limelight.getInstance().setPipeline(Constants.LIMELIGHT_AUTO_PIPELINE);
        Limelight.getInstance().setLedMode(Limelight.LightMode.ON);
        drive.resetOdometry(new Pose2d());
        turret.resetHomePosition(Constants.TURRET_COMPETITION_HOME_POSITION_DEGREES);
        magazine.resetHomePosition();
        shooter.resetHoodHomePosition();
        drive.resetGyroYawAngle(Constants.DRIVE_COMPETITION_GYRO_HOME_ANGLE_DEGREES);
        drive.setControlMode(Drive.DriveControlMode.PATH_FOLLOWING);

        m_autonomousCommand = autonTaskChooser.getSelected();

        /*
         * String autoSelected = SmartDashboard.getString("Auto Selector",
         * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
         * = new MyAutoCommand(); break; case "Default Auto": default:
         * autonomousCommand = new ExampleCommand(); break; }
         */

        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    /**
     * This method is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic()
    {
    }

    @Override
    public void teleopInit()
    {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
        drive.setControlMode(Drive.DriveControlMode.JOYSTICK);
        Limelight.getInstance().setLedMode(Limelight.LightMode.OFF);
      //  shooter.setHoodMotionMagicPositionAbsolute(Constants.HOOD_COMPETITION_HOME_POSITION_DEGREES);
    }

    /**
     * This method is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic()
    {
    }

    @Override
    public void testInit()
    {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /**
     * This method is called periodically during test mode.
     */
    @Override
    public void testPeriodic()
    {
    }
}
