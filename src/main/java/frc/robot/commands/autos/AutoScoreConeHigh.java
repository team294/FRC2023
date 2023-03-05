// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorPosition;
import frc.robot.Constants.WristConstants.WristAngle;
import frc.robot.commands.*;
import frc.robot.commands.sequences.*;
import frc.robot.subsystems.*;
import frc.robot.utilities.FileLog;

public class AutoScoreConeHigh extends SequentialCommandGroup {

  /**
   * Raise elevator to high position, score cone, and lower elevator
   */
  public AutoScoreConeHigh(Elevator elevator, Wrist wrist, Manipulator manipulator, LED led, FileLog log) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new FileLogWrite(true, false, "AutoScoreConeHigh", "Start", log),
      new ManipulatorSetPistonPosition(true, led, manipulator, log),		// set to cone position
      new ManipulatorSetPercent(ManipulatorConstants.pieceGrabPct, manipulator, log),				// Low power to hold piece
      new ElevatorWristMoveToUpperPosition(ElevatorPosition.scoreHighCone.value, WristAngle.scoreMidHigh.value, elevator, wrist, log),
      new WaitCommand(0.25),
      new EjectPiece(manipulator, log), 		// Runs for 1 second
      new ElevatorWristStow(elevator, wrist, log),
      new FileLogWrite(true, false, "AutoScoreConeHigh", "Finish", log)
    );
  }
}
