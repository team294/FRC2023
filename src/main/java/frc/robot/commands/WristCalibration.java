// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.Wrist;
import frc.robot.utilities.FileLog;

public class WristCalibration extends CommandBase {

  private Wrist wrist;
  private FileLog log;
  private double percentOutput, rampRate;
  private int state;
  private final Timer timer = new Timer();
  /**
   * 
   * @param rampRate  Ramp rate in pctOut/second 
   * @param wrist
   * @param log
   */
  /** Creates a new WristCalibration. */
  public WristCalibration(double rampRate, Wrist wrist, FileLog log) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.wrist = wrist;
    this.log = log;
    this.rampRate = rampRate;
    addRequirements(wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    state = 0;

    wrist.enableFastLogging(true);
    log.writeLog(false, "WristCalibration", "Initialize", "rampRate", rampRate);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currTime = timer.get();

    if (!wrist.isEncoderCalibrated()) return;

    switch (state) {
      case 0:   // ramp upwards until 10 inches from top
        if (wrist.getWristAngle() < WristConstants.WristAngle.upperLimit.value - 2.0) {
          percentOutput = MathUtil.clamp(currTime*rampRate, -1.0, 1.0);
          wrist.setWristMotorPercentOutput(percentOutput);      
        } else {
          state = 1;
          timer.reset();
          timer.start();
        }
        break;
      case 1:    // stop motor for 2 seconds, then go to state 2
        if (currTime < 2.0) {
          wrist.stopWrist();
        } else {
          state = 2;
          timer.reset();
          timer.start();
        }
        break;
      case 2:   // ramp downwards until 10 inches from bottom
        if (wrist.getWristAngle() > WristConstants.WristAngle.lowerLimit.value + 2.0) {
          percentOutput = MathUtil.clamp(-currTime*rampRate, -1.0, 1.0);
          wrist.setWristMotorPercentOutput(percentOutput);
        } else {
          state = 3;
          timer.reset();
          timer.start();
        }
        break;
      default:    // stop motor for 2 seconds, then go to state 0
        if (currTime < 2.0) {
          wrist.stopWrist();     
        } else {
          state = 0;
          timer.reset();
          timer.start();
        }
        break;
    }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
