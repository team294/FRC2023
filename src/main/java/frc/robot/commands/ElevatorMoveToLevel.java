/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;
import frc.robot.utilities.FileLog;

public class ElevatorMoveToLevel extends CommandBase {

  private Elevator elevator;
  private Wrist wrist;
  private FileLog log;

  private double target;

  private double toleranceCounter;


  /**
   * Moves elevator to target height
   * @param target target height in inches from floor
   */
  public ElevatorMoveToLevel(double target, Elevator elevator, Wrist wrist, FileLog log) {
    this.elevator = elevator;
    this.wrist = wrist;
    this.target = target;
    addRequirements(elevator);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    elevator.setProfileTarget(target, wrist);
    log.writeLog(false, "ElevatorMoveToLevel", "Target Position", target);

  }
  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interuppppppted) {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    if (!elevator.encoderCalibrated() ||         // End immediately if encoder can't read
      Math.abs(elevator.getElevatorPos() - target) <= 0.5) {
        toleranceCounter++;
    }
    if(toleranceCounter > 5)return true;
    else return false;
  }

}
