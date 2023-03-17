// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.commands.IntakePistonSetPosition;
import frc.robot.commands.IntakeSetPercentOutput;
import frc.robot.commands.ManipulatorGrab;
import frc.robot.commands.ManipulatorGrab.BehaviorType;
import frc.robot.subsystems.*;
import frc.robot.utilities.FileLog;

public class IntakeExtendAndTurnOnMotors extends SequentialCommandGroup {
  /**
   * Extends intake and runs intake and manipulator motors if the elevator is down
   * @param manipulator
   * @param intake
   * @param elevator
   * @param log
   */
  public IntakeExtendAndTurnOnMotors(Manipulator manipulator, Intake intake, Wrist wrist, Elevator elevator, FileLog log) {
    addCommands(
      new ConditionalCommand(
        new SequentialCommandGroup(
          new IntakePistonSetPosition(true, intake, elevator, log),
          new IntakeSetPercentOutput(0.75, .6, intake, log),
          new ElevatorWristStow(elevator, wrist, log),
          new ManipulatorGrab(ManipulatorConstants.pieceGrabPct, BehaviorType.waitForConeOrCube, manipulator, log)
        ),
        new WaitCommand(0.01),
        () -> elevator.isElevatorAtLowerLimit()
      )
    );
  }
}
