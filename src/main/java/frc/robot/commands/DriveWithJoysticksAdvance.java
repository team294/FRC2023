// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utilities.FileLog;


public class DriveWithJoysticksAdvance extends CommandBase {
  Joystick leftJoystick;
  Joystick rightJoystick;
  DriveTrain driveTrain;
  ProfiledPIDController xSpeedController, ySpeedController, turnRateController;
  FileLog log;
  private int logRotationKey;
  private double targetFwdVelocity, targetLeftVelocity, targetTurnRate, nextFwdVelocity, nextLeftVelocity, nextTurnRate;
  private ChassisSpeeds currSpeed;

    /**
   * @param leftJoystick left joystick.  X and Y axis control robot movement, relative to front of robot
   * @param rightJoystick right joystick.  X-axis controls robot rotation.
   * @param maxVelocity max velocity of the robot
   * @param maxAccel max acceleration of the robot
   * @param driveTrain drive train subsystem to use
   * @param log filelog to use
   */

  public DriveWithJoysticksAdvance(Joystick leftJoystick, Joystick rightJoystick, double maxVelocity, double maxAccel, DriveTrain driveTrain, FileLog log) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.leftJoystick = leftJoystick;
    this.rightJoystick = rightJoystick;
    this.driveTrain = driveTrain;
    this.log = log;
    xSpeedController = new ProfiledPIDController(TrajectoryConstants.kPXController, 0, 0, new TrapezoidProfile.Constraints(maxVelocity, maxAccel));
    ySpeedController = new ProfiledPIDController(TrajectoryConstants.kPYController, 0, 0, new TrapezoidProfile.Constraints(maxVelocity, maxAccel));
    turnRateController = new ProfiledPIDController(TrajectoryConstants.kPThetaController, 0, 0, new TrapezoidProfile.Constraints(SwerveConstants.kMaxTurningRadiansPerSecond, SwerveConstants.kMaxAngularAccelerationRadiansPerSecondSquared));
    logRotationKey = log.allocateLogRotation();

    addRequirements(driveTrain);
  }

  @Override
  public void initialize() {
    driveTrain.setDriveModeCoast(false);

    xSpeedController.reset(0.0);
    ySpeedController.reset(0.0);
    turnRateController.reset(0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    targetFwdVelocity = -leftJoystick.getY();
    targetLeftVelocity = -leftJoystick.getX();
    targetTurnRate = -rightJoystick.getX();
    currSpeed = driveTrain.getRobotSpeeds();

    SmartDashboard.putNumber("Left Joystick Y", targetFwdVelocity);
    SmartDashboard.putNumber("Left Joystick X", targetLeftVelocity);
    SmartDashboard.putNumber("Right Joystick X", targetTurnRate);

    // Apply deadbands

    targetFwdVelocity = (Math.abs(targetFwdVelocity) < OIConstants.joystickDeadband) ? 0 : scaleJoystick(targetFwdVelocity) * SwerveConstants.kMaxSpeedMetersPerSecond;
    targetLeftVelocity = (Math.abs(targetLeftVelocity) < OIConstants.joystickDeadband) ? 0 : scaleJoystick(targetLeftVelocity) * SwerveConstants.kMaxSpeedMetersPerSecond;
    targetTurnRate = (Math.abs(targetTurnRate) < OIConstants.joystickDeadband) ? 0 : scaleTurn(targetTurnRate) * SwerveConstants.kMaxTurningRadiansPerSecond;

    // Calculates using the profiledPIDController what the next speed should be

    nextFwdVelocity = xSpeedController.calculate(currSpeed.vxMetersPerSecond, targetFwdVelocity);
    nextLeftVelocity = ySpeedController.calculate(currSpeed.vyMetersPerSecond, targetLeftVelocity);
    nextTurnRate = ySpeedController.calculate(currSpeed.omegaRadiansPerSecond, targetLeftVelocity);



    if(log.isMyLogRotation(logRotationKey)) {
      log.writeLog(false, "DriveWithJoystickAdvance", "Joystick", "Fwd", currSpeed.vxMetersPerSecond, "Left", currSpeed.vyMetersPerSecond, "Turn", targetTurnRate);
    }
    

    driveTrain.drive(nextFwdVelocity, nextLeftVelocity, nextTurnRate, true, false);


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


