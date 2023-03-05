// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorPosition;
import frc.robot.Constants.ElevatorConstants.ElevatorRegion;
import frc.robot.Constants.WristConstants.WristAngle;
import frc.robot.Constants.WristConstants.WristRegion;
import frc.robot.commands.*;
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
      // If elevator is not in main and wrist is in main or down region, then raise elevator
      new ConditionalCommand(
        new ElevatorSetPosition(ElevatorConstants.boundMainLow+3.0, elevator, log),
        new WaitCommand(0.01), 
        () -> (elevator.getElevatorRegion() != ElevatorRegion.main && 
               (wrist.getWristRegion() == WristRegion.main || wrist.getWristRegion() == WristRegion.down)
              )
      ),

      // If elevator is higher than a little into main, then put wrist back before lowering
      new ConditionalCommand(
        new WristSetAngle(WristAngle.elevatorMoving, wrist, log),
        new WaitCommand(0.01), 
        () -> (elevator.getElevatorPos() >= ElevatorConstants.boundMainLow+8.0)
      ),

      // lower elevator to lower edge of main region
      new ElevatorSetPosition(ElevatorConstants.boundMainLow+3.0, elevator, log)
        .until( () -> elevator.getElevatorPos() <= ElevatorConstants.boundMainLow+10.0),

      // move wrist to stow position
      new WristSetAngle(WristAngle.loadConveyor, wrist, log),

      // move elevator to stow position
      new ElevatorSetPosition(ElevatorPosition.bottom, elevator, log)
    );
  }
}
