/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;
import frc.robot.utilities.FileLog;

public class WristSetAngle extends CommandBase {

  private double target;
  private double angle;
  private final Wrist wrist;
  private final FileLog log;
  private final boolean fromShuffleboard;

  /**
   * Moves wrist to target angle
   * @param angle target angle in degrees
   */
  public WristSetAngle(double angle, Wrist wrist, FileLog log) {
    this.angle = angle;
    this.wrist = wrist;
    this.log = log;
    fromShuffleboard = false;

    addRequirements(wrist);
  }

  /**
   * Moves wrist to target angle
   */
  public WristSetAngle(Wrist wrist, FileLog log) {
    this.wrist = wrist;
    this.log = log;
    fromShuffleboard = true;

    if(SmartDashboard.getNumber("Wrist Angle", -9999) == -9999) {
      SmartDashboard.putNumber("Wrist Angle", 0);
    }
    addRequirements(wrist);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    if(fromShuffleboard){
      target = SmartDashboard.getNumber("Wrist Angle", 0);
    }
    wrist.setWristAngle(target + wrist.getCurrentWristTarget());
    log.writeLog(false, "WristSetAngle", "Initialize", "Target", target + wrist.getCurrentWristTarget());
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
