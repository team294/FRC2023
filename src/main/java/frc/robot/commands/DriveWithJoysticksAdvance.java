// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utilities.FileLog;
import frc.robot.utilities.MathBCR;


public class DriveWithJoysticksAdvance extends CommandBase {
  Joystick leftJoystick;
  Joystick rightJoystick;
  DriveTrain driveTrain;
  ProfiledPIDController turnRateController;
  FileLog log;
  private int logRotationKey;
  private double fwdVelocity, leftVelocity, turnRate, nextTurnRate, goalAngle, prevTime, currTime;
  Timer timer;


    /**
   * @param leftJoystick left joystick.  X and Y axis control robot movement, relative to front of robot
   * @param rightJoystick right joystick.  X-axis controls robot rotation.
   * @param driveTrain drive train subsystem to use
   * @param log filelog to use
   */

  public DriveWithJoysticksAdvance(Joystick leftJoystick, Joystick rightJoystick, DriveTrain driveTrain, FileLog log) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.leftJoystick = leftJoystick;
    this.rightJoystick = rightJoystick;
    this.driveTrain = driveTrain;
    this.log = log;
    turnRateController = new ProfiledPIDController(TrajectoryConstants.kPThetaController, 0, 0, new TrapezoidProfile.Constraints(SwerveConstants.kMaxTurningRadiansPerSecond, SwerveConstants.kMaxAngularAccelerationRadiansPerSecondSquared));
    turnRateController.enableContinuousInput(-180, 180);

    logRotationKey = log.allocateLogRotation();

    addRequirements(driveTrain);
  }

  @Override
  public void initialize() {
    driveTrain.setDriveModeCoast(false);


    goalAngle = driveTrain.getPose().getRotation().getDegrees();
    timer = new Timer();
    timer.reset();
    timer.start();
    prevTime = timer.get();

    turnRateController.reset(goalAngle);      // sets the current measurement and the current setpoint for the controller
    turnRateController.setGoal(goalAngle);    // set the goal for the controller
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    fwdVelocity = -leftJoystick.getY();
    leftVelocity = -leftJoystick.getX();
    turnRate = -rightJoystick.getX();
    currTime = timer.get();

    SmartDashboard.putNumber("Left Joystick Y", fwdVelocity);
    SmartDashboard.putNumber("Left Joystick X", leftVelocity);
    SmartDashboard.putNumber("Right Joystick X", turnRate);

    // Apply deadbands

    fwdVelocity = (Math.abs(fwdVelocity) < OIConstants.joystickDeadband) ? 0 : scaleJoystick(fwdVelocity) * SwerveConstants.kMaxSpeedMetersPerSecond;
    leftVelocity = (Math.abs(leftVelocity) < OIConstants.joystickDeadband) ? 0 : scaleJoystick(leftVelocity) * SwerveConstants.kMaxSpeedMetersPerSecond;
    turnRate = (Math.abs(turnRate) < OIConstants.joystickDeadband) ? 0 : scaleTurn(turnRate) * SwerveConstants.kMaxTurningRadiansPerSecond;

    // Calculate goal angle
    goalAngle += turnRate*(currTime-prevTime);
    goalAngle = MathBCR.normalizeAngle(goalAngle);

    // Calculates using the profiledPIDController what the next speed should be

    nextTurnRate = turnRateController.calculate(driveTrain.getPose().getRotation().getDegrees(), goalAngle);



    if(log.isMyLogRotation(logRotationKey)) {
      log.writeLog(false, "DriveWithJoystickAdvance", "Joystick", "Fwd", fwdVelocity, "Left", leftVelocity, "Turn", nextTurnRate);
    }
    

    driveTrain.drive(fwdVelocity, leftVelocity, nextTurnRate, true, false);

    prevTime = currTime;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  /**
   * Re-maps joystick value to better enable fine robot control at small joystick
   * values (low speeds) and full-speed travel at large joystick values.
   * This method is optimized for linear travel.
   * @param rawJoystick Raw joystick value, -1.0 to +1.0
   * @return Scaled joystick value, -1.0 to +1.0
   */
  private double scaleTurn(double rawJoystick){
    return Math.signum(rawJoystick)*(0.6801 * rawJoystick * rawJoystick + 0.3232 * Math.abs(rawJoystick) - 0.0033);
  }

  /**
   * Re-maps joystick value to better enable fine robot control at small joystick
   * values (low speeds) and full-speed travel at large joystick values.
   * This method is optimized for rotating the robot.
   * @param rawJoystick Raw joystick value, -1.0 to +1.0
   * @return Scaled joystick value, -1.0 to +1.0
   */
  private double scaleJoystick(double rawJoystick){
    return Math.signum(rawJoystick)*(0.7912*rawJoystick*rawJoystick + 0.2109*Math.abs(rawJoystick) - 0.0022);
  }
}


