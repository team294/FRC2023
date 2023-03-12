// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.*;
import frc.robot.Constants.ElevatorConstants.ElevatorPosition;
import frc.robot.Constants.WristConstants.WristAngle;
import frc.robot.commands.*;
import frc.robot.commands.sequences.*;
import frc.robot.subsystems.*;
import frc.robot.utilities.FileLog;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoScoreCubeHigh extends SequentialCommandGroup {
  /** Creates a new AutoScoreCubeHigh. */
  public AutoScoreCubeHigh(Pose2d initPose, Pose2d finalPose, DriveTrain driveTrain, Elevator elevator, Wrist wrist, Manipulator manipulator, LED led, FileLog log) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new FileLogWrite(true, false, "AutoScoreCubeHigh", "Start", log),
      new ManipulatorSetPistonPosition(false, led, manipulator, log),		// set to cube position
      new ManipulatorSetPercent(ManipulatorConstants.pieceGrabPct, manipulator, log),				// Low power to hold piece
      new ElevatorWristMoveToUpperPosition(ElevatorPosition.scoreHighCone.value, WristAngle.scoreMidHigh.value, elevator, wrist, log),
      new WaitCommand(0.25),
      new DriveToPose(finalPose, driveTrain, log),
      new EjectPiece(manipulator, log), 		// Runs for 1 second
      new DriveToPose(initPose, driveTrain, log),
      new ElevatorWristStow(elevator, wrist, log),
      new FileLogWrite(true, false, "AutoScoreConeHigh", "Finish", log)
    );
  }
}
