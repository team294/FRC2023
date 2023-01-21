package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Constants.CoordType;
import frc.robot.Constants.StopType;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utilities.FileLog;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;



public class DriveTrajectory extends SequentialCommandGroup { 
    
    
    /**
     * 
     * @param trajectoryType Specify what robot starting position to use
     * kRelative = path starts where robot is, kAbsolute = path starts where it was told to regardless of whether the robot is actually there
     * kAbsoluteResetPose = path starts where it was told to and robot is set at that starting point
     * @param stopAtEnd  True = robot stops at end of trajectory, False = robot does not end stopped
     * @param trajectory The trajectory to 
     * @param driveTrain The driveTrain subsystem to be controlled.
     * @param log        File for logging
     */

    public DriveTrajectory(CoordType trajectoryType, StopType stopAtEnd, Trajectory trajectory, DriveTrain driveTrain, FileLog log){ 

        var thetaController =
            new ProfiledPIDController(
                Constants.TrajectoryConstants.kPThetaController, 0, 0, Constants.TrajectoryConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                trajectory,
                driveTrain::getPose,
                Constants.DriveConstants.kDriveKinematics,
                new PIDController(Constants.TrajectoryConstants.kPXController, 0, 0),
                new PIDController(Constants.TrajectoryConstants.kPYController, 0, 0),
                thetaController,
                (a) -> driveTrain.setModuleStates(a, false),
                driveTrain);

        //if coordType is not absoluteResetPose, then it would be relative pose which is the false condition
        addCommands(
            new ConditionalCommand(
                new InstantCommand(() -> driveTrain.resetPose(trajectory.getInitialPose())),
                new WaitCommand(0),
            () -> trajectoryType == CoordType.kAbsoluteResetPose),


            swerveControllerCommand,
            new ConditionalCommand(
                new DriveStop(driveTrain, log),
                new InstantCommand(() -> driveTrain.setDriveModeCoast(true))
                ,
            () -> stopAtEnd == StopType.kBrake)

        );
    }
}