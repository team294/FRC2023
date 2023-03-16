// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.utilities.FileLog;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeToManipulator extends SequentialCommandGroup {
  /** Creates a new IntakeToManipulator. */
  /**
   *  Extends intake and runs intake motor, Stows Wrist and Elevator, Runs manipulator motor to pick up from ground
   * @param intake
   * @param elevator
   * @param wrist
   * @param manipulator
   * @param log
   */
  public IntakeToManipulator(Intake intake, Elevator elevator, Wrist wrist, Manipulator manipulator, FileLog log) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new IntakePistonSetPosition(true, intake, elevator, log),
      new IntakeSetPercentOutput(0.75, intake, log),
      new ElevatorWristStow(elevator, wrist, log),
      new ManipulatorSetPercent(ManipulatorConstants.pieceGrabPct, manipulator, log)
    );
  }
}
