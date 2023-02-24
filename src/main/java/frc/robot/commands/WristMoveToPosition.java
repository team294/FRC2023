/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.Constants.WristConstants;
import frc.robot.Constants.WristConstants.WristAngle;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;
import frc.robot.utilities.FileLog;

public class WristMoveToPosition extends CommandBase {

  private double target;
  private boolean targetAngle; // true if target is angle, false if target is position
  private WristAngle pos;
  private final Wrist wrist;
  private final Elevator elevator;
  private final FileLog log;

  /**
   * Moves wrist to target angle
   * @param angle target angle in degrees
   */
  public WristMoveToPosition(double angle, Wrist wrist, Elevator elevator, FileLog log) {
    target = angle;
    targetAngle = true;
    this.wrist = wrist;
    this.elevator = elevator;
    this.log = log;

    addRequirements(wrist);
  }

  /**
   * Moves wrist to target position
   * @param pos target position based on WristAngle from RobotPrefs
   */
  public WristMoveToPosition(WristAngle pos, Wrist wrist, Elevator elevator, FileLog log) {
    this.pos = pos;
    targetAngle = true;
    this.wrist = wrist;
    this.elevator = elevator;
    this.log = log;

    addRequirements(wrist);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    if (targetAngle) {
      wrist.setWristAngle(target, elevator);
    } else {
      switch (pos) {
        case stowed:
          wrist.setWristAngle(WristConstants.stowed, elevator);
          break;
        case straight:
          wrist.setWristAngle(WristConstants.straight, elevator);
          break;
        case scoreCargo:
          wrist.setWristAngle(WristConstants.scoreCargo, elevator);
          break;
        case vision:
          wrist.setWristAngle(WristConstants.vision, elevator);
          break;
      }
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    wrist.updateWristLog(false);
  }


  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return !wrist.isEncoderCalibrated() || Math.abs(wrist.getWristAngle() - wrist.getCurrentWristTarget()) < 5.0; // tolerance of 5 degrees
  }
}
