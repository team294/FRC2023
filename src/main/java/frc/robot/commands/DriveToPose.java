// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants.TrajectoryConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utilities.FileLog;

public class DriveToPose extends CommandBase {
  private final DriveTrain driveTrain;
  private final FileLog log;
  private final Pose2d goalPose;
  
  private final Timer timer = new Timer();
  private HolonomicDriveController controller;

  /**
   * Drives the robot to the desired pose in field coordinates.
   * @param goalPose target pose in field coordinates.  Pose components include
   *    <p> Robot X location in the field, in meters (0 = field edge in front of driver station, +=away from our drivestation)
   *    <p> Robot Y location in the field, in meters (0 = right edge of field when standing in driver station, +=left when looking from our drivestation)
   *    <p> Robot angle on the field (0 = facing away from our drivestation, + to the left, - to the right)
   * @param driveTrain DriveTrain subsystem
   * @param log file for logging
   */
  public DriveToPose(Pose2d goalPose, DriveTrain driveTrain, FileLog log) {
    this.driveTrain = driveTrain;
    this.log = log;
    this.goalPose = goalPose;

    addRequirements(driveTrain);

    // Define the controller for robot rotation
    ProfiledPIDController thetaController = new ProfiledPIDController(
        TrajectoryConstants.kPThetaController, 0, 0, TrajectoryConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
    controller = new HolonomicDriveController(
      new PIDController(TrajectoryConstants.kPXController, 0, 0),
      new PIDController(TrajectoryConstants.kPYController, 0, 0),
      thetaController );
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();

    //TODO reset holonomic controller
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(
    //TODO Lots of work....
  ) {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
