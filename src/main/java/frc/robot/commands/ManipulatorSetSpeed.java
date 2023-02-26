// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Manipulator;
import frc.robot.utilities.FileLog;

public class ManipulatorSetSpeed extends CommandBase {
  /** Creates a new ManipulatorStopMotor. */
  private final double speed;
  private final Manipulator manipulator;
  private final FileLog log;
  public enum ManipulatorEnd {
    runForever, stopOnSensor
  }
  private final ManipulatorEnd type;

  public ManipulatorSetSpeed(double speed, ManipulatorEnd type, Manipulator manipulator, FileLog log) {
    this.speed = speed;
    this.manipulator = manipulator;
    this.log = log;
    this.type = type;
    addRequirements(manipulator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    manipulator.setMotorPercentOutput(speed);
    log.writeLog(false, "ManipulatorEject", "Initialize");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    manipulator.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(type == ManipulatorEnd.stopOnSensor){
      if(manipulator.getPistonCone()){
        if(manipulator.isConePresent()){
          return true;
        }else return false;
      }else{
        if(manipulator.isCubePresent()){
          return true;
        }else return false;
      }
    }else return false;
  }
}
