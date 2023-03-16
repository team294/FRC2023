/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.CoordType;
import frc.robot.Constants.WristConstants;;
import frc.robot.subsystems.*;
import frc.robot.utilities.*;

public class WristSetAngleTrapezoid extends CommandBase {
  /**
   * Uses wpilib TrapezoidProfile generator to generate a motion profile for drive train turning
   * Does not regenerate the profile every time
   */

  private Wrist wrist; // reference to driveTrain
  private CoordType coordType;
  private double target; // goal angle
  private double currAngle;
  private double startAngle;
  private double maxVel; // max velocity, between 0 and kMaxSpeedMetersPerSecond in Constants 
  private double maxAccel; // max acceleration, between 0 and kMaxAccelerationMetersPerSecondSquared in Constants
  private long profileStartTime; // initial time (time of starting point)
  private boolean regenerate;
  private FileLog log;
  private boolean isOpenLoop;

  private int accuracyCounter = 0;

  private TrapezoidProfileBCR tProfile; // wpilib trapezoid profile generator
  private TrapezoidProfileBCR.State tStateCurr; // initial state of the system (position in deg and time in sec)
  private TrapezoidProfileBCR.State tStateNext; // next state of the system as calculated by the profile generator
  private TrapezoidProfileBCR.State tStateFinal; // goal state of the system (position in deg and time in sec)
  private TrapezoidProfileBCR.Constraints tConstraints; // max vel (deg/sec) and max accel (deg/sec/sec) of the system
 
  /**
   * Drives the robot straight.
   * @param target distance to travel, in meters (+ = forward, - = backward)
   * @param fieldRelative false = angle is relative to current robot facing,
   *   true = angle is an absolute field angle (0 = away from drive station)
   * @param angle angle to drive along when driving straight (+ = left, - = right)
   * @param maxVel max velocity in meters/second, between 0 and kMaxSpeedMetersPerSecond in Constants
   * @param maxAccel max acceleration in meters/second2, between 0 and kMaxAccelerationMetersPerSecondSquared in Constants
   * @param regenerate true = regenerate profile each cycle (to accurately reach target distance), false = don't regenerate (for debugging)
   * @param isOpenLoop true = feed-forward only for velocity control, false = PID feedback velocity control
   * @param driveTrain reference to the drive train subsystem
   * @param log
   */
  public WristSetAngleTrapezoid(double target, CoordType coordType, double maxVel, double maxAccel, boolean regenerate, boolean isOpenLoop, Wrist wrist, FileLog log) {
    this.wrist = wrist;
    this.log = log;
    this.regenerate = regenerate;
    this.isOpenLoop = isOpenLoop;
    this.target = target;
    this.coordType = coordType;
    this.maxVel = MathUtil.clamp(Math.abs(maxVel), 0, WristConstants.kMaxAngularVel);
    this.maxAccel = MathUtil.clamp(Math.abs(maxAccel), 0, WristConstants.kMaxAngularAcc);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(wrist);
  }


  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Calculate target angle
    if (coordType == CoordType.kAbsolute) {
      target = target;
    }
    
    tStateFinal = new TrapezoidProfileBCR.State(target, 0.0); // initialize goal state (degrees to turn)
    tStateCurr = new TrapezoidProfileBCR.State(0.0, 0.0); // initialize initial state (relative turning, so assume initPos is 0 degrees)
    tConstraints = new TrapezoidProfileBCR.Constraints(maxVel, maxAccel); // initialize velocity and accel limits
    tProfile = new TrapezoidProfileBCR(tConstraints, tStateFinal, tStateCurr); // generate profile
    log.writeLog(false, "WristSetAngleTrapezoid", "init", "Target", target, "Profile total time", tProfile.totalTime());
    
    profileStartTime = System.currentTimeMillis(); // save starting time of profile
    startAngle = wrist.getWristAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Update data for this iteration
    long currProfileTime = System.currentTimeMillis();
    double timeSinceStart = (double)(currProfileTime - profileStartTime) * 0.001;
    currAngle = driveTrain.getPose().getTranslation().getDistance(startAngle);

    // Get next state from trapezoid profile
    tStateNext = tProfile.calculate(timeSinceStart + 0.010);
    double targetVel = tStateNext.velocity;
    double targetAccel = tStateNext.acceleration;

    // Set wheel speeds
    // Note:  All 4 SwerveModuleStates in the desiredStates[] array point to the same SwerveModuleState object.
    // So, they will always have the same speed, even if we only update one of the elements of the array.
    desiredStates[0].speedMetersPerSecond = targetVel;
    desiredStates[1].speedMetersPerSecond = targetVel;
    desiredStates[2].speedMetersPerSecond = targetVel;
    desiredStates[3].speedMetersPerSecond = targetVel;
    driveTrain.setModuleStates(desiredStates, isOpenLoop); 
    
    // Read current module states for logging
    SwerveModuleState[] currentStates = driveTrain.getModuleStates();
    double linearVel = (Math.abs(currentStates[0].speedMetersPerSecond) + Math.abs(currentStates[1].speedMetersPerSecond) +
        Math.abs(currentStates[2].speedMetersPerSecond) + Math.abs(currentStates[3].speedMetersPerSecond))/4.0;

    log.writeLog(false, "WristSetAngleTrapezoid", "profile", "angT", angleTarget,
      "posT", tStateNext.position, 
      "velT", targetVel, "accT", targetAccel,
      "posA", currAngle, 
      "velA", linearVel,
      "velA-FL", currentStates[0].speedMetersPerSecond, 
      "velA-FR", currentStates[1].speedMetersPerSecond, 
      "velA-BL", currentStates[2].speedMetersPerSecond, 
      "velA-BR", currentStates[3].speedMetersPerSecond
    );

    if(regenerate) {
      tStateCurr = new TrapezoidProfileBCR.State(currAngle, linearVel);
      tProfile = new TrapezoidProfileBCR(tConstraints, tStateFinal, tStateCurr);
      profileStartTime = currProfileTime;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    log.writeLog(false, "WristSetAngleTrapezoid", "End");
    driveTrain.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(target - currAngle) < 0.0125) {
      accuracyCounter++;
      log.writeLog(false, "WristSetAngleTrapezoid", "WithinTolerance", "Target Dist", target, "Actual Dist", currAngle, "Counter", accuracyCounter);
    } else {
      accuracyCounter = 0;
    }

    return (accuracyCounter >= 5);
  }
}