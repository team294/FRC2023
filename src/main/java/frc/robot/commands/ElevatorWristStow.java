// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorPosition;
import frc.robot.Constants.ElevatorConstants.ElevatorRegion;
import frc.robot.Constants.WristConstants.WristAngle;
import frc.robot.Constants.WristConstants.WristRegion;
import frc.robot.subsystems.*;
import frc.robot.utilities.FileLog;

public class ElevatorWristStow extends SequentialCommandGroup {


  /**
   * Stows the elevator and wrist in the starting position.  Appropriately moves the
   * wrist and elevator regardless of starting configuration
   * @param elevator
   * @param wrist
   * @param log
   */
  public ElevatorWristStow(Elevator elevator, Wrist wrist, FileLog log) {
    addCommands(
      // If wrist is in main or down region, then raise elevator (if needed) and move wrist into BackMid region
      new ConditionalCommand(
        Commands.sequence(
          new ConditionalCommand(
            new ElevatorSetPosition(ElevatorConstants.boundMainLow+1.0, elevator, log), 
            new WaitCommand(0.01), 
            () -> (elevator.getElevatorRegion() != ElevatorRegion.main)
          ),
          new WristSetAngle(WristConstants.boundBackMidDown-1.0, wrist, log)
        ), 
        new WaitCommand(0.01), 
        () -> (wrist.getWristRegion() == WristRegion.main || wrist.getWristRegion() == WristRegion.down)
      ),
      // move elevator and wrist to final position
      Commands.parallel(
        new ElevatorSetPosition(ElevatorPosition.bottom, elevator, log),
        new WristSetAngle(WristAngle.loadConveyor, wrist, log)
      )
    );
  }
}
