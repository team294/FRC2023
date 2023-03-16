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
import frc.robot.commands.ManipulatorSetPercent;
import frc.robot.commands.ManipulatorGrab.BehaviorType;
import frc.robot.subsystems.*;
import frc.robot.utilities.FileLog;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeExtendAndTurnOnMotors extends SequentialCommandGroup {
  /** Creates a new IntakeExtendAndTurnOnMotors. */
  /**
   * Extends intake and runs intake and manipulator motors if the elevator is down
   * @param manipulator
   * @param intake
   * @param elevator
   * @param log
   */
  public IntakeExtendAndTurnOnMotors(Manipulator manipulator, Intake intake, Elevator elevator, FileLog log) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SequentialCommandGroup(
      new IntakePistonSetPosition(true, intake, elevator, log),
      new IntakeSetPercentOutput(0.75, intake, log),
      new ManipulatorGrab(ManipulatorConstants.pieceGrabPct, BehaviorType.waitForConeOrCube, manipulator, log)
      )
    );
  }
}
