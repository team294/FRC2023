// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utilities.FileLog;
import frc.robot.utilities.HolonomicDriveControllerBCR;
import frc.robot.utilities.TrapezoidProfileBCR;

public class DriveToPose extends CommandBase {
  private final DriveTrain driveTrain;
  private final FileLog log;
  
  private final Timer timer = new Timer();
  private SwerveDriveKinematics kinematics;
  private HolonomicDriveControllerBCR controller;

  private final Supplier<Pose2d> goalSupplier;    // Supplier for goalPose.  null = use passed in goalPose, non-null = use supplier
  private Pose2d initialPose, goalPose, desiredPose;     // Starting, destination, and current desired robot pose (location and rotation) on the field
  private TrapezoidProfileBCR profile;      // Relative linear distance/speeds from initial pose to goal pose 
  private TrapezoidProfileBCR.State curState, goalState;

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

    // Get the goal pose, if using a supplier in the constructor
    if (!(goalSupplier == null)) {
      goalPose = goalSupplier.get();
    }
    
    // Create the profile.  The profile is linear distance relative to the initial pose
    curState = new TrapezoidProfileBCR.State(0, 0);     // TODO get initial velocity along the profile direction from driveTrain odometry?
    goalState = new TrapezoidProfileBCR.State(0000000, 0);    // TODO get distance travelled
    profile = new TrapezoidProfileBCR(TrajectoryConstants.kDriveProfileConstraints, null, null);    //TODO finish this code

    log.writeLog(false, "DriveToPose", "Initialize", 
    "Time", timer.get(), 
    "Goal X", goalPose.getTranslation().getX(),
    "Goal Y", goalPose.getTranslation().getY(),
    "Goal rot", goalPose.getRotation().getDegrees(), 
    "Robot X", initialPose.getTranslation().getX(),
    "Robot Y", initialPose.getTranslation().getY(),
    "Robot rot", initialPose.getRotation().getDegrees()
);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double curTime = timer.get();
    Pose2d robotPose = driveTrain.getPose().relativeTo(initialPose);    // relative??????
    Trajectory.State desiredState = m_trajectory.sample(curTime);        //TODO use trapezoid instead of trajectory (curState, )
    // desiredPose
    double desiredVelocityMetersPerSecond = 0;
    var desiredRotation = goalPose.getRotation();

    var targetChassisSpeeds =
        controller.calculate(robotPose, desiredPose, desiredVelocityMetersPerSecond, desiredRotation);
    var targetModuleStates = kinematics.toSwerveModuleStates(targetChassisSpeeds);

    driveTrain.setModuleStates(targetModuleStates, false);

    ChassisSpeeds robotSpeeds = driveTrain.getChassisSpeeds();
    log.writeLog(false, "DriveToPose", "Execute", 
        "Time", timer.get(), 
        "Trap X", desiredState.poseMeters.getTranslation().getX(),
        "Trap Y", desiredState.poseMeters.getTranslation().getY(),
        "Trap Vel", desiredState.velocityMetersPerSecond,
        "Trap ang", desiredState.poseMeters.getRotation().getDegrees(),
        "Target rot", desiredRotation.getDegrees(), 
        "Robot X", robotPose.getTranslation().getX(),
        "Robot Y", robotPose.getTranslation().getY(),
        "Robot Vel", Math.hypot(robotSpeeds.vyMetersPerSecond, robotSpeeds.vxMetersPerSecond),
        "Robot VelAng", Math.toDegrees(Math.atan2(robotSpeeds.vyMetersPerSecond, robotSpeeds.vxMetersPerSecond)),
        "Robot rot", robotPose.getRotation().getDegrees()
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    log.writeLog(false, "DriveToPose", "End"); 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;     //TODO finish this code
  }
}
