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

    // Save initial pose for relative trajectories
    private Pose2d initialPose;
    
    
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

        // Define the controller for robot rotation
        ProfiledPIDController thetaController =
            new ProfiledPIDController(
                Constants.TrajectoryConstants.kPThetaController, 0, 0, Constants.TrajectoryConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // Create the serve controller command to follow the trajectory
        // Also add any initial commands before following the trajectory, depending on trajectoryType
        SwerveControllerCommand swerveControllerCommand;
        if (trajectoryType == CoordType.kRelative) {
            // For relative trajectories, first command needs to be to save the initial robot Pose
            addCommands(new InstantCommand(() -> initialPose = driveTrain.getPose()));

            swerveControllerCommand =
                new SwerveControllerCommand(
                    trajectory,
                    // For relative trajectories, get the current pose relative to the initial robot Pose
                    () -> driveTrain.getPose().relativeTo(initialPose),  
                    Constants.DriveConstants.kDriveKinematics,
                    new PIDController(Constants.TrajectoryConstants.kPXController, 0, 0),
                    new PIDController(Constants.TrajectoryConstants.kPYController, 0, 0),
                    thetaController,
                    (a) -> driveTrain.setModuleStates(a, false),
                    driveTrain);
        } else {
            if (trajectoryType == CoordType.kAbsoluteResetPose) {
                // For AbsoluteResetPose trajectories, first command needs to be to reset the robot Pose
                addCommands(new InstantCommand(() -> driveTrain.resetPose(trajectory.getInitialPose())));
            }

            swerveControllerCommand =
                new SwerveControllerCommand(
                    trajectory,
                    driveTrain::getPose,
                    Constants.DriveConstants.kDriveKinematics,
                    new PIDController(Constants.TrajectoryConstants.kPXController, 0, 0),
                    new PIDController(Constants.TrajectoryConstants.kPYController, 0, 0),
                    thetaController,
                    (a) -> driveTrain.setModuleStates(a, false),
                    driveTrain);
        }

        // Next follow the trajectory
        addCommands(swerveControllerCommand);
        
        // Add any final commands, per the stopAtEnd
        if (stopAtEnd == StopType.kBrake) {
            addCommands(new InstantCommand(() -> driveTrain.setDriveModeCoast(false)),
                new DriveStop(driveTrain, log)
            );
        } else if (stopAtEnd == StopType.kCoast) {
            addCommands(new InstantCommand(() -> driveTrain.setDriveModeCoast(true)),
                new DriveStop(driveTrain, log)
            );
        }
    }

}