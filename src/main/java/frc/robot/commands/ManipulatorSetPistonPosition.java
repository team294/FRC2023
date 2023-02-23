// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Manipulator;
import frc.robot.utilities.FileLog;

public class ManipulatorSetPistonPosition extends CommandBase {
  /** Creates a new ManipulatorStopMotor. */
  private final boolean cone;
  private final Manipulator manipulator;
  private final FileLog log;

  private final LED led = new LED();
  /**
   * Sets piston position to handle either cone or cube
   * @param cone true = cone, false = cube
   * @param manipulator manipulator subsystem
   * @param log filelog
   */
  public ManipulatorSetPistonPosition(boolean cone, Manipulator manipulator, FileLog log) {
    this.cone = cone;
    this.manipulator = manipulator;
    this.log = log;

    addRequirements(manipulator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    manipulator.setPistonCone(cone);
    if(cone){
      new LEDSetStrip("Yellow", led, log);
    } else {
      new LEDSetStrip("Purple", led, log);
    }
    log.writeLog(false, "ManipulatorSetPistonCone", "Initialize");
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
    return true;
  }
}
