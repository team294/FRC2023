/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utilities.FileLog;

public class DriveResetPose extends CommandBase {
  /**
   * Resets the pose, gyro, and encoders on the drive train
   */

  private DriveTrain driveTrain;
  private FileLog log;
  private double curX, curY, curAngle;    // in meters and degrees
  private boolean fromShuffleboard;
  private boolean onlyAngle;      // true = resent angle but not X-Y position

  /**
	 * Resets the pose, gyro, and encoders on the drive train
   * @param curXinMeters Robot X location in the field, in meters (0 = middle of robot wherever the robot starts auto mode, +=away from our drivestation)
   * @param curYinMeters Robot Y location in the field, in meters (0 = middle of robot wherever the robot starts auto mode, +=left when looking from our drivestation)
   * @param curAngleinDegrees Robot angle on the field, in degrees (0 = facing away from our drivestation)
   * @param driveTrain DriveTrain subsytem
   * @param log FileLog
	 */
  public DriveResetPose(double curXinMeters, double curYinMeters, double curAngleinDegrees, DriveTrain driveTrain, FileLog log) {
    this.driveTrain = driveTrain;
    this.log = log;
    curX = curXinMeters;
    curY = curYinMeters;
    curAngle = curAngleinDegrees;
    fromShuffleboard = false;
    onlyAngle = false;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  /**
	 * Resets the pose, gyro, and encoders on the drive train.
   * Reset the angle but keep the current position (use the current measured position as the new position).
   * @param curAngleinDegrees Robot angle on the field, in degrees (0 = facing away from our drivestation)
   * @param driveTrain DriveTrain subsytem
   * @param log FileLog
	 */
  public DriveResetPose(double curAngleinDegrees, DriveTrain driveTrain, FileLog log) {
    this.driveTrain = driveTrain;
    this.log = log;
    curAngle = curAngleinDegrees;
    fromShuffleboard = false;
    onlyAngle = true;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  /**
   * esets the pose, gyro, and encoders on the drive train.  Gets values from Shuffleboard
   * @param driveTrain DriveTrain subystem
   * @param log FileLog
   */
  public DriveResetPose(DriveTrain driveTrain, FileLog log){
    this.driveTrain = driveTrain;
    this.log = log;
    fromShuffleboard = true;
    onlyAngle = false;

    addRequirements(driveTrain);
    
    if(SmartDashboard.getNumber("DriveResetPose X", -9999) == -9999) {
      SmartDashboard.putNumber("DriveResetPose X", 0);
    }
    if(SmartDashboard.getNumber("DriveResetPose Y", -9999) == -9999) {
      SmartDashboard.putNumber("DriveResetPose Y", 0);
    }
    if(SmartDashboard.getNumber("DriveResetPose Angle", -9999) == -9999){
      SmartDashboard.putNumber("DriveResetPose Angle", 0);
    }
    

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(fromShuffleboard){
      curX = SmartDashboard.getNumber("DriveResetPose X", 0);
      curX = SmartDashboard.getNumber("DriveResetPose Y", 0);
      curAngle = SmartDashboard.getNumber("DriveResetPose Angle", 0);
    }

    if(onlyAngle){
      curX = driveTrain.getPose().getX();
      curY = driveTrain.getPose().getY();
    }
    
    log.writeLog(false, "DriveResetPose", "Init", "X", curX, "Y", curY, "Angle", curAngle);

    driveTrain.resetPose(new Pose2d(curX, curY, Rotation2d.fromDegrees(curAngle)));
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
