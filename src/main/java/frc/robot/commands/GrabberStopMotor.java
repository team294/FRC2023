// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Grabber;
import frc.robot.utilities.FileLog;

public class GrabberStopMotor extends CommandBase {
  /** Creates a new GrabberStopMotor. */
  private final Grabber grabber;
  private final FileLog log;
  
  public GrabberStopMotor(Grabber grabber, FileLog log) {
    this.grabber = grabber;
    this.log = log;

    addRequirements(grabber);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    grabber.stopMotor();
    log.writeLog(false, "GrabberStopMotor", "Initialize");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
