// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.utilities.FileLog;

public class IntakePistonSetPosition2 extends CommandBase {
  Intake intake;
  FileLog log;
  boolean deploy;
  /**
   * Sets intake pistion position
   * @param intake intake subsystem
   * @param boolean true = deploy, false = undeploy
   * @param log log file
   */
  public IntakePistonSetPosition2(Intake intake, boolean deploy, FileLog log) {
    this.intake  = intake;
    this.deploy = deploy;
    this.log = log;
    addRequirements(intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.setDeployed(deploy);
    log.writeLog(false, intake.getName(), "IntakeSetPiston", "IntakePiston", (deploy) ? "Deploy" : "Retract");
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
