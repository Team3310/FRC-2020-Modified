package frc.robot.auto.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;


public class ShootWhileDriving extends ParallelCommandGroup {

    public ShootWhileDriving(Drive drive, Shooter shooter, Turret turret, Magazine magazine){
        addCommands(

                new LimelightSetLED(Limelight.getInstance(), Limelight.LightMode.ON),
                new InstantCommand(()->Limelight.getInstance().setPipeline(Constants.LIMELIGHT_ALL_FIELD_PIPELINE)),
                new ShooterSetReady(shooter,false),
                new ShooterSetRPM(shooter, Constants.SHOOTER_MAIN_ALL_FIELD_RPM, Constants.SHOOTER_KICKER_ALL_FIELD_RPM),
                new MagazineSetRPM(magazine, Constants.MAGAZINE_SHOOT_RPM),
                new TurretSetLagAngle(drive, turret),
                new HoodSetAngleFromDist(shooter,drive.getRobotDistanceFromGoal())



        );

    }
}
