// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Grabber;
import frc.robot.utilities.FileLog;


public class GrabberPickUp extends CommandBase {

  private final Grabber grabber;
  private final FileLog log;
  private double percent = -0.5;
  // private boolean spiked = false;
  /**
   * Sets the motor speed from Shuffboard, -1.0 to +1.0
   * @param motor motor subsystem
   * @param log logfile
   */
  public GrabberPickUp(Grabber grabber, FileLog log) {
    this.grabber = grabber;
    this.log = log;
    SmartDashboard.putNumber("Percent", percent);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(grabber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // spiked = false;
    percent = -0.5;
    log.writeLog(false, "GrabberPickUp", "Start");
    grabber.setMotorPercentOutput(percent);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Percent", percent);
    if(grabber.getAmps() > 10/*  || spiked*/){
      // if (!spiked) {
      //   percent = .2;
      // }
      // spiked = true;
      log.writeLog(false, "GrabberPickUp", "Amps", grabber.getAmps(), "Percent", percent);
      percent += .0005;
      grabber.setMotorPercentOutput(percent);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    grabber.setMotorPercentOutput(-0.1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return percent >= -.1;
  }
}
