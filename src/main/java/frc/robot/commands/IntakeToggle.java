// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.utilities.FileLog;

public class IntakeToggle extends CommandBase {
  /** Creates a new IntakeRetract. */
  private Intake intake;
  private FileLog log;

  
  /**
   * toggles intake between deployed and undeployed
   * @param intake 
   * @param log
   */
  public IntakeToggle(Intake intake, FileLog log) {
    this.intake = intake;
    this.log = log;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    log.writeLog(false, "IntakeToggle", "Initialize", "Extended at start", intake.isDeployed());

    // turn off/on intake before closing/opening intake
    if (intake.isDeployed()) {
      intake.setMotorPercentOutput(0);
      intake.setDeployed(false);
    } else {
      intake.setDeployed(true);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
