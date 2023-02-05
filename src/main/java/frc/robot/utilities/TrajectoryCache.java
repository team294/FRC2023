// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveConstants;

/**
 * Class that defines and caches all trajectories that the robot could run.
 * Create one object instance of this class when the robot initializes to build the trajectories. 
 */
public class TrajectoryCache {
    private FileLog log;
   
    private static int trajectoryCount = 2;
    public Trajectory[] cache = new Trajectory[trajectoryCount];

    public enum TrajectoryType {
        test(0),
        testCurve(1),
        CenterBalanceAuto(2),
        LeaveCommuniy(3),
        OuterOneConeBalanceLeftAuto(4),
        OuterOneConeBalanceMiddleAuto(5),
        OuterOneConeBalanceRightAuto(6),
        Pickup(7);
        @SuppressWarnings({"MemberName", "PMD.SingularField"})
        public final int value;
        TrajectoryType(int value) { this.value = value; }
    }
	
    /**
     * Build all trajectories in this.cache[] for trajectory-following commands.
     * @param log
     */
    public TrajectoryCache(FileLog log){
        this.log = log;

        cache[TrajectoryType.test.value] = calcTrajectory("Test", 0.4, 0.4, false, 
            new Pose2d(0, 0, new Rotation2d(0.0)),
            List.of(),
            new Pose2d(6.0, 0, new Rotation2d(Math.toRadians(0.0)))
        );

        cache[TrajectoryType.testCurve.value] = calcTrajectory("Test Curve", 0.4, 0.4, false, 
            new Pose2d(0, 0, new Rotation2d(0.0)),
            List.of(),
            new Pose2d(3, 3, new Rotation2d(Math.toRadians(90.0)))
        );     
        
        cache[TrajectoryType.CenterBalanceAuto.value] = calcTrajectory("CenterBalanceAuto", 0.4, 0.4, false, 
            // Start at the position (1.75895, 2.707) facing +X direction
            new Pose2d(1.75895, 2.707, new Rotation2d(0)),
            List.of(),
            // Go straight onto platform
            new Pose2d(5.75895, 2.707, new Rotation2d(0))
        );    

        cache[TrajectoryType.LeaveCommuniy.value] = calcTrajectory("LeaveCommuniy", 0.4, 0.4, false, 
            // Start at the origin facing the +X direction
            new Pose2d(0.0, 0.0, new Rotation2d(0)),
            List.of(
            ),
            new Pose2d(-3.168, 0, new Rotation2d(Math.toRadians(Math.PI)))
            // Pass through these two interior waypoints, making an 's' curve path
            // List.of(new Translation2d(-1, 1), new Translation2d(2, -1)),
        ); 

        cache[TrajectoryType.OuterOneConeBalanceLeftAuto.value] = calcTrajectory("OuterOneConeBalanceLeftAuto", 0.4, 0.4, false, 
            new Pose2d(0, 0, new Rotation2d(0)),
            // 
            List.of(new Translation2d(-4.11, 0), new Translation2d(-4.11, 1.37)),
            // Pass through these two interior waypoints, making an 's' curve path

            new Pose2d(-2.4, 1.37, new Rotation2d(0))
        ); 

        cache[TrajectoryType.OuterOneConeBalanceMiddleAuto.value] = calcTrajectory("OuterOneConeBalanceMiddleAuto", 0.4, 0.4, false, 
            new Pose2d(0, 0, new Rotation2d(0)),
            // 
            List.of(new Translation2d(-4.11, 0)),
            // Pass through these two interior waypoints, making an 's' curve path

            new Pose2d(-2.4, 0, new Rotation2d(0))
        ); 

        cache[TrajectoryType.OuterOneConeBalanceRightAuto.value] = calcTrajectory("OuterOneConeBalanceRightAuto", 0.4, 0.4, false, 
        new Pose2d(0, 0, new Rotation2d(0)),
        // 
        List.of(new Translation2d(-4.11, 0), new Translation2d(-4.11, -1.37)),
        // Pass through these two interior waypoints, making an 's' curve path

        new Pose2d(-2.4, -1.37, new Rotation2d(0))
        ); 

        cache[TrajectoryType.Pickup.value] = calcTrajectory("Pickup", 0.4, 0.4, false, 
        new Pose2d(0.0, 0.0, new Rotation2d(0)),
        List.of(),
        // new Pose2d(-6.7, 0, new Rotation2d(Math.PI))
        // new Pose2d(0, 0, new Rotation2d(0)),
        new Pose2d(-3, 0, new Rotation2d(Math.PI))
        ); 
    }


    /**
     * Builds a single trajectory based on the parameters passed in:
     * @param trajName name of the trajectory
     * @param maxVelRatio maximum velocity multiplier between 0 and 1
     * @param maxAccelRatio maximum acceleration multiplier between 0 and 1
     * @param setReversed true = robot drives backwards, false = robot drives forwards
     * @param startPose Pose2d starting position (coordinates and angle)
     * @param interriorWaypoints List of Translation 2d waypoints (just coordinates)
     * @param endPose Pose2d ending position (coordinates and angle)
     * @return trajectory that is generated
     */
    private Trajectory calcTrajectory(String trajName, double maxVelRatio, double maxAccelRatio, 
        boolean setReversed, Pose2d startPose, List<Translation2d> interriorWaypoints, Pose2d endPose) {
		Trajectory trajectory = null;
	
    	try {

			log.writeLogEcho(true, "TrajectoryGeneration", trajName, 
				"maxSpeed", SwerveConstants.kMaxSpeedMetersPerSecond * maxVelRatio,
				"maxAcceleration", SwerveConstants.kMaxAccelerationMetersPerSecondSquare * maxAccelRatio);

			// Create config for trajectory
            TrajectoryConfig config = new TrajectoryConfig(SwerveConstants.kMaxSpeedMetersPerSecond * maxVelRatio,
				SwerveConstants.kMaxAccelerationMetersPerSecondSquare * maxAccelRatio)
				.setKinematics(DriveConstants.kDriveKinematics)
				.setReversed(setReversed);			// Set to true if robot is running backwards

            // Generate the trajectory
			trajectory = TrajectoryGenerator.generateTrajectory(
				startPose, interriorWaypoints, endPose, config);

			// debug logging
			TrajectoryUtil.dumpTrajectory(trajectory, log);

		} catch (Exception e) {
			log.writeLogEcho(true, "TrajectoryGeneration", trajName, 
				"ERROR in calcTrajectory", e.toString(),"exception",e);
		}

		if (trajectory != null) {
			log.writeLogEcho(true, "TrajectoryGeneration", trajName, "SUCCESS", true);
		};
	
		return trajectory;
	}

}
