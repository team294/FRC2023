// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.commands.sequences.*;
import frc.robot.subsystems.*;
import frc.robot.utilities.FileLog;

public class AutoScoreCube extends SequentialCommandGroup {

  /**
   * Scores a cube, including driving the robot to the scoring position, elevator/wrist moves to score,
   * ejecting the cube, and stowing the elevator/wrist.
   * @param initPose  Slightly backed off from scoring position on field
   * @param finalPose  Scoring position on field
   * @param elevatorPosition Scoring position for elevator (such as ElevatorPosition.scoreHighCone.value)
   * @param wristAngle Scoring position for wrist (such as WristAngle.scoreMidHigh.value)
   * @param driveTrain
   * @param elevator
   * @param wrist
   * @param manipulator
   * @param intake
   * @param led
   * @param log
   */
  public AutoScoreCube(Pose2d initPose, Pose2d finalPose, double elevatorPosition, double wristAngle,
      DriveTrain driveTrain, Elevator elevator, Wrist wrist, 
      Manipulator manipulator, Intake intake, LED led, FileLog log) {
    addCommands(
      new FileLogWrite(true, false, "AutoScoreCube", "Start", log, "Elevator pos", elevatorPosition, "Wrist angle", wristAngle),
      new ManipulatorSetPistonPosition(false, led, manipulator, log),		// set to cube position
      new ManipulatorSetPercent(ManipulatorConstants.pieceHoldPct, manipulator, log),				// Low power to hold piece
      new ElevatorWristMoveToUpperPosition(elevatorPosition, wristAngle, elevator, wrist, intake, log),
      new DriveToPose(finalPose, driveTrain, log),
      new EjectPiece(manipulator, log), 		// Runs for 1 second
      new DriveToPose(initPose, driveTrain, log),
      new ElevatorWristStow(elevator, wrist, log),
      new FileLogWrite(true, false, "AutoScoreCube", "Finish", log)
    );
  }
}
