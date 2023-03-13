// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.CoordType;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.utilities.FileLog;
import frc.robot.utilities.MathBCR;

public class AutoBalance extends SequentialCommandGroup {

  /**
   * Balaces the robot in autonomous.  First goes to absolute coords of posCommunityInitial (make sure path
   * for robot from starting location to this location is clear!), then  TODO
   * @param posCommunityInitial
   * @param posCommunityFinal
   * @param driveTrain
   * @param log
   */
  public AutoBalance(Pose2d posCommunityInitial, Pose2d posCommunityFinal, DriveTrain driveTrain, FileLog log) {
    Pose2d posCommunityFarther, posCommunityCloser;

    posCommunityFinal = MathBCR.translate(posCommunityFinal, 1.5, 0.0);

    posCommunityFarther = MathBCR.translate(posCommunityFinal, 2.0, 0.0);
    posCommunityCloser = MathBCR.translate(posCommunityFinal, -2.0, 0.0);

    addCommands(
      new DriveToPose(posCommunityInitial, driveTrain, log),
      new DriveToPose(posCommunityFinal, 2.0, SwerveConstants.kNominalAccelerationMetersPerSecondSquare, 
        TrajectoryConstants.maxPositionErrorMeters, TrajectoryConstants.maxThetaErrorDegrees, false, driveTrain, log),

      new ConditionalCommand(
        new DriveToPose(posCommunityCloser, 0.3, SwerveConstants.kNominalAccelerationMetersPerSecondSquare, 
          TrajectoryConstants.maxPositionErrorMeters, TrajectoryConstants.maxThetaErrorDegrees, false, driveTrain, log)
            .until(() -> driveTrain.getGyroPitch() < DriveConstants.maxPitchBalancedDegrees), // drive forward slowly
        new WaitCommand(0.01), 
        () -> driveTrain.getGyroPitch() > DriveConstants.maxPitchBalancedDegrees
      ),
      new ConditionalCommand(
        new DriveToPose(posCommunityFarther, 0.3, SwerveConstants.kNominalAccelerationMetersPerSecondSquare, 
          TrajectoryConstants.maxPositionErrorMeters, TrajectoryConstants.maxThetaErrorDegrees, false, driveTrain, log)
            .until(() -> driveTrain.getGyroPitch() > -DriveConstants.maxPitchBalancedDegrees), // drive forward slowly
        new WaitCommand(0.01), 
        () -> driveTrain.getGyroPitch() < -DriveConstants.maxPitchBalancedDegrees
      ),

      new DriveToPose(CoordType.kRelative, 0.5, driveTrain, log)		// Lock the wheels at 45deg
    );
  }
}
