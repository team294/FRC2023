/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.CoordType;
import frc.robot.Constants.WristConstants;
import frc.robot.Constants.WristConstants.WristAngle;
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
  private double maxVel; // max velocity, between 0 and kMaxAngularVel in Constants 
  private double maxAccel; // max acceleration, between 0 and kMaxAngularAcc in Constants
  private long profileStartTime; // initial time (time of starting point)
  // private boolean regenerate;
  private boolean fromShuffleboard;
  private FileLog log;

  private int accuracyCounter = 0;

  private TrapezoidProfileBCR tProfile; // wpilib trapezoid profile generator
  private TrapezoidProfileBCR.State tStateCurr; // initial state of the system (position in deg and time in sec)
  private TrapezoidProfileBCR.State tStateNext; // next state of the system as calculated by the profile generator
  private TrapezoidProfileBCR.State tStateFinal; // goal state of the system (position in deg and time in sec)
  private TrapezoidProfileBCR.Constraints tConstraints; // max vel (deg/sec) and max accel (deg/sec/sec) of the system
 
  /**
   * Sets Wrist angle with trapezoid profile.
   * @param target Angle to go to(- is down, + is up)
   * @param coordType relative, relative to the current angle of the wrist,
   *   absolute, angle is absolute to the actual wrist angle of the wrist
   * @param maxVel max angular velocity in degrees/second, between 0 and kMaxAngularVel in Constants
   * @param maxAccel max acceleration in degrees/second2, between 0 and kMaxAngularAcc in constants
   * @param wrist reference to the wrist subsystem
   * @param log
   */
  public WristSetAngleTrapezoid(double target, CoordType coordType, double maxVel, double maxAccel, Wrist wrist, FileLog log) {
    this.wrist = wrist;
    this.log = log;
    // this.regenerate = regenerate;
    this.target = target;
    this.coordType = coordType;
    this.maxVel = MathUtil.clamp(Math.abs(maxVel), 0, WristConstants.kMaxAngularVel);
    this.maxAccel = MathUtil.clamp(Math.abs(maxAccel), 0, WristConstants.kMaxAngularAcc);
    fromShuffleboard = false;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(wrist);
  }

  /**
   * Drives the robot straight.
   * @param coordType relative, relative to the current angle of the wrist,
   *   absolute, angle is absolute to the actual wrist angle of the wrist
   * @param maxVel max angular velocity in degrees/second, between 0 and kMaxAngularVel in Constants
   * @param maxAccel max acceleration in degrees/second2, between 0 and kMaxAngularAcc in constants
   * @param wrist reference to the wrist subsystem
   * @param log
   */
  public WristSetAngleTrapezoid(Wrist wrist, FileLog log) {
    this.wrist = wrist;
    this.log = log;
    // this.regenerate = regenerate;
    target = 0;
    this.coordType = CoordType.kAbsolute;
    this.maxVel = 0.5 * WristConstants.kMaxAngularVel;
    this.maxAccel = 0.5*WristConstants.kMaxAngularAcc;
    fromShuffleboard = true;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(wrist);
    if(SmartDashboard.getNumber("WristSetAngleTrapezoid Manual Target Angle", -9999) == -9999) {
      SmartDashboard.putNumber("WristSetAngleTrapezoid Manual Target Angle", 2);
    }
    if(SmartDashboard.getNumber("WristSetAngleTrapezoid Manual MaxVel", -9999) == -9999) {
      SmartDashboard.putNumber("WristSetAngleTrapezoid Manual MaxVel", 0.5*WristConstants.kMaxAngularVel);
    }
    if(SmartDashboard.getNumber("WristSetAngleTrapezoid Manual MaxAccel", -9999) == -9999) {
      SmartDashboard.putNumber("WristSetAngleTrapezoid Manual MaxAccel", 0.5*WristConstants.kMaxAngularAcc);
    }
  }


  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(fromShuffleboard) {
      target = SmartDashboard.getNumber("WristSetAngleTrapezoid Manual Target Angle", 2);
      maxVel = SmartDashboard.getNumber("WristSetAngleTrapezoid Manual MaxVel", WristConstants.kMaxAngularVel);
      maxVel = MathUtil.clamp(Math.abs(maxVel), 0, WristConstants.kMaxAngularVel);
      maxAccel = SmartDashboard.getNumber("WristSetAngleTrapezoid Manual MaxAccel", WristConstants.kMaxAngularAcc);
      maxAccel = MathUtil.clamp(Math.abs(maxAccel), 0, WristConstants.kMaxAngularAcc);
    }
    // change to relative angle
    if (coordType == CoordType.kAbsolute) {
      target = target - wrist.getWristAngle();
    }
    // Sets wrist limits
    if(wrist.getWristAngle() + target > WristAngle.upperLimit.value){
      target = WristAngle.upperLimit.value - wrist.getWristAngle();
    }
    else if(wrist.getWristAngle() + target < WristAngle.lowerLimit.value){
      target = WristAngle.lowerLimit.value - wrist.getWristAngle();
    }

        // Calculate target angle

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
    currAngle = wrist.getWristAngle() - startAngle;

    // Get next state from trapezoid profile
    tStateNext = tProfile.calculate(timeSinceStart + 0.010);
    double targetVel = tStateNext.velocity;
    double targetAccel = tStateNext.acceleration;

    wrist.setWristVelocity(targetVel);

    log.writeLog(false, "WristSetAngleTrapezoid", "profile", "angT", target,
      "posT", tStateNext.position, 
      "velT", targetVel, "accT", targetAccel,
      "posA", currAngle
    );

    // if(regenerate) {
      // tStateCurr = new TrapezoidProfileBCR.State(currAngle, );
      // tProfile = new TrapezoidProfileBCR(tConstraints, tStateFinal, tStateCurr);
      // profileStartTime = currProfileTime;
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    log.writeLog(false, "WristSetAngleTrapezoid", "End");
    // wrist.setWristAngle(wrist.getWristAngle());
    wrist.setWristMotorPercentOutput(WristConstants.kG * Math.cos(wrist.getWristAngle() *Math.PI/180.0));
    // wrist.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(target - currAngle) < 3) {
      accuracyCounter++;
      log.writeLog(false, "WristSetAngleTrapezoid", "WithinTolerance", "Target Dist", target, "Actual Angle", currAngle, "Counter", accuracyCounter);
    } else {
      accuracyCounter = 0;
    }

    return (accuracyCounter >= 5);
  }
}