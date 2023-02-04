// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utilities.FileLog;
import frc.robot.utilities.HolonomicDriveControllerBCR;
import frc.robot.utilities.Translation2dBCR;
import frc.robot.utilities.TrapezoidProfileBCR;

public class DriveToPose extends CommandBase {
  private final DriveTrain driveTrain;
  private final FileLog log;
  
  private final Timer timer = new Timer();
  private SwerveDriveKinematics kinematics;
  private HolonomicDriveControllerBCR controller;

  private final Supplier<Pose2d> goalSupplier;    // Supplier for goalPose.  null = use passed in goalPose, non-null = use supplier
  private Pose2d initialPose, goalPose;     // Starting and destination robot pose (location and rotation) on the field
  private Translation2d initialTranslation;     // Starting robot translation on the field
  private Translation2d goalDirection;          // Unit vector pointing from initial pose to goal pose = direction of travel
  private TrapezoidProfileBCR profile;      // Relative linear distance/speeds from initial pose to goal pose 

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
    goalSupplier = null;

    constructorCommonCode();
  }

  /**
   * Drives the robot to the desired pose in field coordinates.
   * @param goalPoseSupplier A function that supplies the target pose in field coordinates.  Pose components include
   *    <p> Robot X location in the field, in meters (0 = field edge in front of driver station, +=away from our drivestation)
   *    <p> Robot Y location in the field, in meters (0 = right edge of field when standing in driver station, +=left when looking from our drivestation)
   *    <p> Robot angle on the field (0 = facing away from our drivestation, + to the left, - to the right)
   * @param driveTrain DriveTrain subsystem
   * @param log file for logging
   */
  public DriveToPose(Supplier<Pose2d> goalPoseSupplier, DriveTrain driveTrain, FileLog log) {
    this.driveTrain = driveTrain;
    this.log = log;
    goalSupplier = goalPoseSupplier;

    constructorCommonCode();
  }

  /**
   * Common code between multiple constructors
   */
  private void constructorCommonCode() {
    addRequirements(driveTrain);

    // Define the swerve drive kinematics
    kinematics = Constants.DriveConstants.kDriveKinematics;

    // Define the controller for robot rotation
    ProfiledPIDController thetaController = new ProfiledPIDController(
        TrajectoryConstants.kPThetaController, 0, 0, TrajectoryConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
    // Define the holomic controller to controll driving
    controller = new HolonomicDriveControllerBCR(
      new PIDController(TrajectoryConstants.kPXController, 0, 0),
      new PIDController(TrajectoryConstants.kPYController, 0, 0),
      thetaController );
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Reset timer and controllers
    timer.reset();
    timer.start();
    controller.reset();

    // Get the initial pose
    initialPose = driveTrain.getPose();
    initialTranslation = initialPose.getTranslation();

    // Get the goal pose, if using a supplier in the constructor
    if (!(goalSupplier == null)) {
      goalPose = goalSupplier.get();
    }

    // Calculate the direction and distance of travel
    Translation2d trapezoidPath = goalPose.getTranslation().minus(initialTranslation);
    goalDirection = Translation2dBCR.normalize(trapezoidPath);
    double goalDistance = trapezoidPath.getNorm();
    
    // Get the initial velocity in the direction of travel
    ChassisSpeeds robotSpeed = driveTrain.getRobotSpeeds();
    double initialVelocity = robotSpeed.vxMetersPerSecond*goalDirection.getX() + robotSpeed.vyMetersPerSecond*goalDirection.getY();

    // Create the profile.  The profile is linear distance (along goalDirection) relative to the initial pose
    TrapezoidProfileBCR.State initialState = new TrapezoidProfileBCR.State(0, initialVelocity);
    TrapezoidProfileBCR.State goalState = new TrapezoidProfileBCR.State(goalDistance, 0);
    profile = new TrapezoidProfileBCR(TrajectoryConstants.kDriveProfileConstraints, goalState, initialState);

    log.writeLog(false, "DriveToPose", "Initialize", 
      "Time", timer.get(), 
      "Goal X", goalPose.getTranslation().getX(),
      "Goal Y", goalPose.getTranslation().getY(),
      "Goal rot", goalPose.getRotation().getDegrees(), 
      "Robot X", initialTranslation.getX(),
      "Robot Y", initialTranslation.getY(),
      "Robot rot", initialPose.getRotation().getDegrees()
    );
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double curTime = timer.get();

    // Current robot location, translation is relative to starting position, rotation is absolute field rotation
    Translation2d robotTranslation = driveTrain.getPose().getTranslation().minus(initialTranslation);
    Pose2d robotPose = new Pose2d(robotTranslation, Rotation2d.fromDegrees(driveTrain.getGyroRotation()));

    // Calculate current desired pose and velocity from the Trapezoid profile, relative to starting position
    TrapezoidProfileBCR.State desiredState = profile.calculate(curTime);
    Pose2d desiredPose = new Pose2d( goalDirection.times(desiredState.position), goalDirection.getAngle());
    double desiredVelocityMetersPerSecond = desiredState.velocity;
    Rotation2d desiredRotation = goalPose.getRotation();

    ChassisSpeeds targetChassisSpeeds =
        controller.calculate(robotPose, desiredPose, desiredVelocityMetersPerSecond, desiredRotation);
    var targetModuleStates = kinematics.toSwerveModuleStates(targetChassisSpeeds);

    driveTrain.setModuleStates(targetModuleStates, false);

    ChassisSpeeds robotSpeeds = driveTrain.getRobotSpeeds();
    log.writeLog(false, "DriveToPose", "Execute", 
        "Time", timer.get(), 
        "Trap X", desiredPose.getTranslation().getX(),
        "Trap Y", desiredPose.getTranslation().getY(),
        "Trap Vel", desiredVelocityMetersPerSecond,
        "Trap VelAng", desiredPose.getRotation().getDegrees(),
        "Target rot", desiredRotation.getDegrees(), 
        "Robot X", robotTranslation.getX(),
        "Robot Y", robotTranslation.getY(),
        "Robot Vel", Math.hypot(robotSpeeds.vyMetersPerSecond, robotSpeeds.vxMetersPerSecond),
        "Robot VelAng", Math.toDegrees(Math.atan2(robotSpeeds.vyMetersPerSecond, robotSpeeds.vxMetersPerSecond)),
        "Robot rot", robotPose.getRotation().getDegrees()
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    log.writeLog(false, "DriveToPose", "End"); 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(profile.totalTime())  && 
        ( Math.abs(driveTrain.getGyroRotation() - goalPose.getRotation().getDegrees()) <= TrajectoryConstants.maxThetaErrorDegrees );
  }
}
