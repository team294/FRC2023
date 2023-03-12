// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorPosition;
import frc.robot.Constants.WristConstants.WristAngle;
import frc.robot.commands.*;
import frc.robot.commands.ManipulatorGrab.BehaviorType;
import frc.robot.subsystems.*;
import frc.robot.utilities.FileLog;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveToLoad extends SequentialCommandGroup {
  /** Creates a new DriveToLoad. */
  public DriveToLoad(DriveTrain driveTrain, Wrist wrist, Elevator elevator, Manipulator manipulator, FileLog log) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DriveToPose(new Pose2d(16.17878-1.7, 6.749796-0.25, new Rotation2d(0)),
      SwerveConstants.kMaxSpeedMetersPerSecond, SwerveConstants.kMaxAccelerationMetersPerSecondSquare,
      TrajectoryConstants.interimPositionErrorMeters, TrajectoryConstants.interimThetaErrorDegrees,  driveTrain, log),
      new DriveStop(driveTrain, log),
      new ManipulatorGrab(0.8, BehaviorType.immediatelyEnd, manipulator, log),
      new ConditionalCommand(
        new ElevatorWristMoveToUpperPosition(ElevatorPosition.loadingStationCone.value, WristAngle.loadHumanStation.value, elevator, wrist, log), 
        new ElevatorWristMoveToUpperPosition(ElevatorPosition.loadingStationCube.value, WristAngle.loadHumanStation.value, elevator, wrist, log), 
        manipulator::getPistonCone
      ),
      // new ManipulatorGrab(0.8, BehaviorType.immediatelyEnd, manipulator, log),
      new DriveToPose(new Pose2d(16.17878-1.6, 6.749796-0.25, new Rotation2d(0)), driveTrain, log).until(() -> (manipulator.isConePresent() || manipulator.isCubePresent())),
      new ManipulatorGrab(0.8, BehaviorType.waitForConeOrCube, manipulator, log),
      new DriveStop(driveTrain, log)
      // new ManipulatorGrab(0.8, BehaviorType.waitForConeOrCube, manipulator, log)
      );
  }
}
