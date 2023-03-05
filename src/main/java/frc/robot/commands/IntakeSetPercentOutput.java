// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.utilities.FileLog;

public class IntakeSetPercentOutput extends CommandBase {
  /** Creates a new IntakeSetPercentOutput. */
  private Intake intake;
  private double intakePercentOutput;
  private FileLog log;

  /**
   * Sets the percent output of the intake motor, + is intake, - is outtake
   * @param intakePercentOutput -1.0 to +1.0
   * @param intake intake subsystem
   * @param log log file
   */
  public IntakeSetPercentOutput(double intakePercentOutput, Intake intake, FileLog log) {
    this.intake = intake;
    this.intakePercentOutput = intakePercentOutput;
    this.log = log;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    log.writeLog(false, "IntakeSetPercentOutput", "Initialize" ,"Intake Percent", intakePercentOutput);

    intake.setMotorPercentOutput(intakePercentOutput);
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
