/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.WristConstants;
import frc.robot.Constants.WristConstants.WristAngle;
import frc.robot.subsystems.Wrist;
import frc.robot.utilities.FileLog;

public class WristMoveToPosition extends CommandBase {

  private double target;
  private final Wrist wrist;
  private final FileLog log;

  /**
   * Moves wrist to target angle
   * @param angle target angle in degrees
   */
  public WristMoveToPosition(double angle, Wrist wrist, FileLog log) {
    target = angle;
    this.wrist = wrist;
    this.log = log;

    addRequirements(wrist);
  }

  /**
   * Moves wrist to target position
   * @param pos target position based on WristAngle from Constants
   */
  public WristMoveToPosition(WristAngle pos, Wrist wrist, FileLog log) {
    target = pos.value;
    this.wrist = wrist;
    this.log = log;

    addRequirements(wrist);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    wrist.setWristAngle(target);

    log.writeLog(false, "WristMoveToPosition", "Initialize", "Target", target);
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
