package frc.robot.commands.sequences;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorPosition;
import frc.robot.Constants.WristConstants.WristAngle;
import frc.robot.commands.*;
import frc.robot.commands.ManipulatorGrab.BehaviorType;
import frc.robot.subsystems.*;
import frc.robot.utilities.Field;
import frc.robot.utilities.FileLog;

public class DriveToLoad extends SequentialCommandGroup {

  /**
   * ADD A DESCRIPTION HERE
   * @param driveTrain
   * @param wrist
   * @param elevator
   * @param manipulator
   * @param log
   */
  public DriveToLoad(DriveTrain driveTrain, Wrist wrist, Elevator elevator, Manipulator manipulator, Field field, FileLog log) {
    addCommands(
      //TODO This is only coded for one color team right now!!!  Which team?  Code for both teams.
      new DriveToPose(field.getLoadingPositionInitial(),
        SwerveConstants.kFullSpeedMetersPerSecond, SwerveConstants.kFullAccelerationMetersPerSecondSquare,
        TrajectoryConstants.interimPositionErrorMeters, TrajectoryConstants.interimThetaErrorDegrees, true, driveTrain, log),
      new DriveStop(driveTrain, log),
      new ManipulatorGrab(0.8, BehaviorType.immediatelyEnd, manipulator, log),
      new ConditionalCommand(
        new ElevatorWristMoveToUpperPosition(ElevatorPosition.loadingStationCone.value, WristAngle.loadHumanStation.value, elevator, wrist, log), 
        new ElevatorWristMoveToUpperPosition(ElevatorPosition.loadingStationCube.value, WristAngle.loadHumanStation.value, elevator, wrist, log), 
        manipulator::getPistonCone
      ),
      // new ManipulatorGrab(0.8, BehaviorType.immediatelyEnd, manipulator, log),
      new DriveToPose(field.getLoadingPositionFinal(), driveTrain, log).until(() -> (manipulator.isConePresent() || manipulator.isCubePresent())),
      new ManipulatorGrab(0.8, BehaviorType.waitForConeOrCube, manipulator, log),
      new DriveStop(driveTrain, log)
      // new ManipulatorGrab(0.8, BehaviorType.waitForConeOrCube, manipulator, log)
      );
  }
}
