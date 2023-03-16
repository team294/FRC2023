// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorRegion;
import frc.robot.Constants.WristConstants.WristAngle;
import frc.robot.Constants.WristConstants.WristRegion;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.utilities.FileLog;

public class ElevatorWristMoveToUpperPosition extends SequentialCommandGroup {
  // public enum ScorePosition {
  //   scoreLow,
  //   scoreMid,
  //   scoreHigh
  // }

  /**
   * Moves the elevator and wrist to an upper position.  Appropriately moves the
   * wrist and elevator regardless of starting configuration to get to the requested configuration.
   * @param elevatorPosition target height in inches, per ElevatorConstants.ElevatorPosition
   * @param wristAngle target angle in degrees.  (0 = horizontal in front of robot, + = up, - = down)
   * @param elevator
   * @param wrist
   * @param intake
   * @param log
   */
  public ElevatorWristMoveToUpperPosition(double elevatorPosition, double wristAngle, Elevator elevator, Wrist wrist, Intake intake, FileLog log) {
    if (elevatorPosition<ElevatorConstants.boundMainLow) {
      elevatorPosition = ElevatorConstants.boundMainLow + 1.0;
    }
    if (wristAngle<WristConstants.boundBackFarMid) {
      wristAngle = WristConstants.boundBackFarMid + 1.0;
    }

    addCommands(
      // Only extend the elevator if the intake is not deployed
      new ConditionalCommand(
        
        new SequentialCommandGroup(
          // If wrist is in backFar region, then move wrist into BackMid region
          // so that the elevator can move up
          new ConditionalCommand(
            new WristSetAngle(WristConstants.boundBackFarMid+1.0, wrist, log), 
            new WaitCommand(0.01), 
            () -> (wrist.getWristRegion() == WristRegion.backFar)
          ),

          // If elevator is not in the main region (and wrist is not in main region), 
          // then move elevator to main region so that the wrist is free to move forward
          new ConditionalCommand(
            new ElevatorSetPosition(ElevatorConstants.boundMainLow+2.0, elevator, log), 
            new WaitCommand(0.01), 
            () -> (elevator.getElevatorRegion() != ElevatorRegion.main && wrist.getWristRegion() != WristRegion.main)
          ),

          // move wrist to safe travel position
          new WristSetAngle(WristAngle.elevatorMoving, wrist, log),

          // move elevator to final position
          new ElevatorSetPosition(elevatorPosition, elevator, log),

          // move wrist to final position
          new WristSetAngle(wristAngle, wrist, log)
        ),
        
        // Don't extend if the intake is deployed
        new WaitCommand(.01),
        () -> !intake.isDeployed()
      )
    );
  }
}
