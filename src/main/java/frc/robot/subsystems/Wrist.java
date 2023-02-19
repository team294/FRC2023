/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.WristConstants;
import frc.robot.Constants.Ports;
import frc.robot.utilities.FileLog;
import frc.robot.utilities.Loggable;
import frc.robot.utilities.Wait;
import static frc.robot.utilities.StringUtil.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Wrist extends SubsystemBase implements Loggable{
  private CANSparkMax wristMotor = new CANSparkMax(Ports.CANWristMotor, MotorType.kBrushless);
  private SparkMaxPIDController wristPIDController;
  // private TalonFXSensorCollection wristLimits;
  private RelativeEncoder relativeEncoder;

  private SparkMaxLimitSwitch revLimitSwitch;
  private SparkMaxLimitSwitch fwdLimitSwitch;
  FileLog log;
  private int logRotationKey;         // key for the logging cycle for this subsystem
  boolean fastLogging;

  private String subsystemName;

  // Don't think we need these
	// private int posMoveCount = 0; // increments every cycle the wrist moves up
	// private int negMoveCount = 0; // increments every cycle the wrist moves down
	// private double currEnc = 0.0; // current recorded encoder value
	// private double encSnapShot = 0.0; // snapshot of encoder value used to make sure encoder is working
  
  private double encoderDegreesPerTicks = 360.0 / WristConstants.encoderTicksPerRevolution;
  private double encoderTicksPerDegrees = WristConstants.encoderTicksPerRevolution / 360.0;

  private boolean usingPosition = false;
  // kP = (desired-output-1.0max)*1024 / (error-in-encoder-ticks)
  // kP = 2.5 -> output of 0.139 when error is 5 degrees
  private double kP = 2.5;  // was 1.0 with original wrist, 2.5 is better with new wrist
  // kI = (desired-output-1.0max)*1024 / [(time-ms) * (error-in-encoder-ticks)]
  // kI = 0.036 -> output of 0.2 when error is 5 degrees for 100ms
	private double kI = 0.0;      // Try 0.036?
  // kD = (desired-output-1.0max)*1024 * (time-ms) / (error-in-encoder-ticks)
  // kD = 200 -> output of 0.2 when error is changing by 90 degrees per second

  // TODO Calibrate
	private double kD = 0.0;  // was 5.0
  private double kFF = 0.0;   // FF gain is multiplied by sensor value (probably in encoder ticks) and divided by 1024
  private double kFFconst = 0.075;   // Add about 1V (0.075* 12V) feed foward constant
  private double kIz = 10;    // Izone in degrees
  // private double kIAccumMax = 0.3/kI;     // Max Iaccumulator value, in encoderTicks*milliseconds.  Max I power = kI * kIAccumMax.
  private double kMaxOutput = 0.6; // up max output
  private double kMinOutput = -0.6; // down max output
  // private double rampRate = 0.3;

  public double wristCalZero;   		// Wrist encoder position at O degrees, in encoder ticks (i.e. the calibration factor)
  public boolean wristCalibrated = false;     // Default to wrist being uncalibrated.  Calibrate from robot preferences or "Calibrate Wrist Zero" button on dashboard
  public double elevatorWristSafeStow; 	

  private double safeAngle;

  public Wrist(String name, FileLog log) {
    this.log = log;
    logRotationKey = log.allocateLogRotation();     // Get log rotation for this subsystem
    fastLogging = false;
    this.subsystemName = name;
    wristPIDController = wristMotor.getPIDController();
    wristMotor.setInverted(true);
    wristMotor.clearFaults();
    wristMotor.setIdleMode(IdleMode.kCoast);


    // wristMotor.set(ControlMode.PercentOutput, 0);
    // wristMotor.setInverted(true);
    // wristMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);
    // wristMotor.configFeedbackNotContinuous(true, 0);
    // wristMotor.setSensorPhase(false);         // Flip sign of sensor reading
    // wristMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    // wristMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    // wristMotor.setNeutralMode(NeutralMode.Brake);
    // wristMotor.configVoltageCompSaturation(12.0);
    // wristMotor.enableVoltageCompensation(true);
    // wristMotor.clearStickyFaults();

    // wristMotor.config_kP(0, kP);
		// wristMotor.config_kI(0, kI);
		// wristMotor.config_kD(0, kD);
		// wristMotor.config_kF(0, kFF);
    // wristMotor.config_IntegralZone(0, (int)degreesToEncoderTicks(kIz));
    // wristMotor.configMaxIntegralAccumulator(0, kIAccumMax);
		// wristMotor.configClosedloopRamp(rampRate);
		// wristMotor.configPeakOutputForward(kMaxOutput);
    // wristMotor.configPeakOutputReverse(kMinOutput);
    
    // wristLimits =  wristMotor.getSensorCollection();
    wristPIDController = wristMotor.getPIDController();
    relativeEncoder = wristMotor.getEncoder();

    // Set PID Coefficients
    wristPIDController.setI(kI);
    wristPIDController.setD(kD);
    wristPIDController.setP(kP);
    wristPIDController.setIZone(kIz);
    wristPIDController.setFF(kFF);
    wristPIDController.setOutputRange(kMinOutput, kMaxOutput);

      // wristPIDController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
      // wristPIDController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
      // wristPIDController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
      // wristPIDController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

    // Wait 0.25 seconds before adjusting the wrist calibration.  The reason is that .setInverted (above)
    // changes the sign of read encoder value, but that change can be delayed up to 50ms for a round trip
    // from the Rio to the Talon and back to the Rio.  So, reading angles could give the wrong value if
    // we don't wait (random weird behavior).
    // DO NOT GET RID OF THIS WITHOUT TALKING TO DON OR ROB.
    Wait.waitTime(250);
    adjustWristCalZero();
  }

  /**
   * Returns the name of the subsystem
   */
  public String getName() {
    return subsystemName;
  }

  /**
   * Sets percent power of wrist motor
   * @param percentPower between -1.0 (down full speed) and 1.0 (up full speed)
   */
  public void setWristMotorPercentOutput(double percentOutput) {
    percentOutput = (percentOutput>kMaxOutput ? kMaxOutput : percentOutput);
    percentOutput = (percentOutput<kMinOutput ? kMinOutput : percentOutput);  

    if (log.isMyLogRotation(logRotationKey)) {
      log.writeLog(false, subsystemName , "Percent Output", "Percent Output", percentOutput);
    }
    wristMotor.set(percentOutput);
  }

  /**
   * Stops wrist motor
   */
  public void stopWrist() {
    wristMotor.stopMotor();
  }

  /**
   * Only works when encoder is working and calibrated
   * If setting to greater than wristKeepOut, elevator position must be at the bottom
   * and target must be at the bottom.
   * @param angle target angle, in degrees (0 = horizontal in front of robot, + = up, - = down)
   */
  public void setWristAngle(double angle, Elevator elevator) {
    if (wristCalibrated) {
      // Don't move wrist in or out of KeepOut if climber > climbWristMovingSafe or elevator > elevatorWristSafeStow.
      if (
            // Not needed because elevator doesn'it interfere with wrist
            /*(elevator.getElevatorPos() > elevatorWristSafeStow ||         // Elevator is not safe
            elevator.getCurrentElevatorTarget() > elevatorWristSafeStow) // Elevator is moving to not safe
            &&*/ (angle > WristConstants.max || angle < WristConstants.min /*|| getWristAngle() > WristConstants.wristKeepOut*/)) {  // We are moving in or out of KeepOut region
        log.writeLog(false, subsystemName, "Set angle", "Angle", angle , "Set angle,N/A,Interlock,Forbidden",
           ",Elevator Position", elevator.getElevatorPos() + 
          ",Elevator Target," + elevator.getCurrentElevatorTarget() + ",Wrist Angle," + getWristAngle());
        return;
      }
      // Angle should be safe as long as its between min and max limits
      // Shouldn't be affected by elevator position
      safeAngle = angle;

      // Apply interlocks if elevator is low
      // if (elevator.getElevatorPos() < ElevatorConstants.groundCargo - 2.0 || elevator.getCurrentElevatorTarget() < ElevatorConstants.groundCargo -2.0) {
      //   // Elevator is very low or is going very low
      //   // Wrist can not be below vision level
      //   safeAngle = (safeAngle < WristConstants.wristVision) ? WristConstants.wristVision : safeAngle;
      // } else {
        // Wrist is safe to move as far down as wristDown
        // safeAngle = (safeAngle < WristConstants.wristDown) ? WristConstants.wristDown : safeAngle;
      // }

      // wristMotor.set(ControlMode.Position, degreesToEncoderTicks(safeAngle) + Robot.robotPrefs.wristCalZero);
      // wristMotor.set(ControlMode.Position, degreesToEncoderTicks(safeAngle) + wristCalZero, 
      //                DemandType.ArbitraryFeedForward, kFFconst);
      // Need usingPosition boolean because there is no way to get controltype from neo
      usingPosition = true; // Starting to set angle with position
      //use arbitrary kFF value?
      wristPIDController.setReference(degreesToEncoderTicks(safeAngle) + wristCalZero, ControlType.kPosition/*, 0, kFFconst*/);
      usingPosition = false; //Finished setting angle with position
      log.writeLog(false, subsystemName, "Set angle", "Desired angle", angle, "Set angle", safeAngle, "Interlock,Allowed",
       "Elevator Pos", elevator.getElevatorPos(), "Elevator Target", elevator.getCurrentElevatorTarget());  
    }
  }

  /**
	 * Sets wrist angle calibration factor and enables angle control modes for wrist
	 * 
	 * @param wristCalZero  Calibration factor for wrist
	 * @param writeCalToPreferences  true = store calibration in Robot Preferences, false = don't change Robot Preferences
	 */
	public void setWristCalibration(double wristCalZero /*boolean writeCalToPreferences,*/) {
		this.wristCalZero = wristCalZero;
		wristCalibrated = true;
		stopWrist();	// Stop motor, so it doesn't jump to new value
		log.writeLog(false, "Preferences", "Calibrate wrist", "zero value", wristCalZero, 
			"Enc Raw", getWristEncoderTicksRaw(),
			"Wrist Angle", getWristAngle(), "Wrist Target", getCurrentWristTarget());
		// if (writeCalToPreferences) {
		// 	prefs.putDouble("wristCalZero", wristCalZero);
		// }
	}

	/**
	 * Stops wrist motor and sets wristCalibrated to false
	 */
	public void setWristUncalibrated() {
		stopWrist();
		log.writeLog(false, "Preferences", "Uncalibrate wrist", "Enc Raw", getWristEncoderTicksRaw(),
			"Wrist Angle", getWristAngle() + ",Wrist Target," + getCurrentWristTarget());
		wristCalibrated = false;
	}

  /**
   * Calibrates the wrist encoder, assuming we know the wrist's current angle
   * @param angle current angle that the wrist is physically at, in degrees (0 = horizontal in front of robot, + = up, - = down)
   * @param saveToPrefs true = save the calibration to RobotPreferences
   */
  public void calibrateWristEnc(double angle /*boolean saveToPrefs,*/) {
    setWristCalibration(getWristEncoderTicksRaw() - degreesToEncoderTicks(angle) /*saveToPrefs,*/);
    setDefaultCommand(null);
  }  
  
	/**
	 * If the angle is reading >/< max/min angle, add/subtract 360 degrees to the wristCalZero accordingly
	 * Note: when the motor is not inverted, upon booting up, an absolute encoder reads a value between 0 and 4096
	 * 		 when the motor is inverted, upon booting up, an absolute encoder reads a value between 0 and -4096
	 * Note: absolute encoder values don't wrap during operation
	 */
	public void adjustWristCalZero() {
    log.writeLogEcho(false, subsystemName, "Adjust wrist pre", "wrist angle", getWristAngle(), 
      "raw ticks", getWristEncoderTicksRaw(), "wristCalZero", wristCalZero);
		if(getWristAngle() < WristConstants.min - 15.0) {
      log.writeLogEcho(false, subsystemName, "Adjust wrist", "Below min angle");
			wristCalZero -= WristConstants.encoderTicksPerRevolution;
		}
		else if(getWristAngle() > WristConstants.max + 10.0) {
      log.writeLogEcho(false, subsystemName, "Adjust wrist", "Above max angle");
			wristCalZero += WristConstants.encoderTicksPerRevolution;
		}
    log.writeLogEcho(false, subsystemName, "Adjust wrist post", "wrist angle", getWristAngle(), 
      "raw ticks", getWristEncoderTicksRaw(), "wristCalZero", wristCalZero);
	}

  /**
   * Reads whether wrist is at lower limit
   * @return true if wrist is at lower limit, false if not
   */
  public boolean getWristLowerLimit() {
    // return wristLimits.isRevLimitSwitchClosed() == 1;
    revLimitSwitch = wristMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
    return revLimitSwitch.isPressed();
  }

  /**
   * Reads whether wrist is at upper limit
   * @return true if wrist is at upper limit, false if not
   */
  public boolean getWristUpperLimit() {
    // return wristLimits.isFwdLimitSwitchClosed() == 1;
    fwdLimitSwitch = wristMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
    return fwdLimitSwitch.isPressed();
  }

  /**
   * 
   * @return raw encoder ticks (based on encoder zero being at horizontal position)
   */
  public double getWristEncoderTicks() {
    if(log.isMyLogRotation(logRotationKey)){
      log.writeLog(false, subsystemName, "Wrist Encoder Ticks", "Wrist Encoder Ticks," + (getWristEncoderTicksRaw() - wristCalZero));
    }
    return getWristEncoderTicksRaw() - wristCalZero;
  }

  /**
   * 
   * @return raw encoder ticks, adjusted direction (positive is towards stowed, negative is towards lower hard stop)
   */
  public double getWristEncoderTicksRaw() {
    // return wristMotor.getSelectedSensorPosition(0);
    return relativeEncoder.getPosition();
  }

  /**
   * 
   * @param encoderTicks encoder ticks
   * @return parameter encoder ticks converted to equivalent degrees
   */
  public double encoderTicksToDegrees(double encoderTicks) {
    return encoderTicks * encoderDegreesPerTicks;
  }

  /**
   * 
   * @param degrees angle in degrees
   * @return parameter degrees converted to equivalent encoder ticks
   */
  public double degreesToEncoderTicks(double degrees) {
    return degrees * encoderTicksPerDegrees;
  }

  /**
   * For use in the wrist subsystem only.  Use getWristAngle() when calling from outside this class.
   * @return current encoder ticks (based on zero) converted to degrees
   */
  private double getWristEncoderDegrees() {
    if(log.isMyLogRotation(logRotationKey)){
      log.writeLog(false, subsystemName, "Wrist Encoder Degrees", "Wrist Encoder Degrees," + encoderTicksToDegrees(getWristEncoderTicks()));
    }
    return encoderTicksToDegrees(getWristEncoderTicks());
  }

  /**
   * 
   * @return current encoder ticks converted to degrees
   */
  public double getWristEncoderDegreesRaw() {
    return encoderTicksToDegrees(getWristEncoderTicksRaw());
  }

  /**
	 * Returns the angle that wrist is currently positioned at in degrees.
	 * If the wrist is not calibrated, then returns wristMax in keepout region to engage all interlocks,
   * since we really don't know where the wrist is at.
	 * @return current degree of wrist angle
	 */
  public double getWristAngle() {
    if (wristCalibrated) {
      double wristAngle = getWristEncoderDegrees();
      wristAngle = wristAngle % 360; // If encoder wraps around 360 degrees
      wristAngle = (wristAngle > 180) ? wristAngle - 360 : wristAngle; // Change range to -180 to +180
      // wristAngle = (wristAngle <= -180) ? wristAngle + 360 : wristAngle; // Change range to -180 to +180  THIS LINE OF CODE DOESN'T WORK!!!!
      if (log.isMyLogRotation(logRotationKey)){
        log.writeLog(false, subsystemName, "Get Wrist Angle", "Wrist Angle", wristAngle);
      }
      return wristAngle;
    } else {
      // Wrist is not calibrated.  Assume we are at max angle in keepout region to engage all interlocks,
      // since we really don't know where the wrist is at.
      return WristConstants.max;
    }
  }

  /**
	 * Returns the angle that wrist is trying to move to in degrees.
	 * If the wrist is not calibrated, then returns wristMax in keepout region to engage all interlocks,
   * since we really don't know where the wrist is at.  If the wrist is in manual control mode, then
   * returns the actual wrist position.
	 * @return desired degree of wrist angle
	 */
  public double getCurrentWristTarget() {
    double currentTarget;

    if (wristCalibrated) {
      if (usingPosition) {
        currentTarget = safeAngle;
      } else {
        // If we are not in position control mode, then we aren't moving towards a target (and the target
        // angle may be undefined).  So, get the actual wrist angle instead.
        currentTarget = getWristAngle();
      }

      currentTarget = currentTarget % 360; // If encoder wraps around 360 degrees
      currentTarget = (currentTarget > 180) ? currentTarget - 360 : currentTarget; // Change range to -180 to +180

      if(log.isMyLogRotation(logRotationKey)){
        log.writeLog(false, subsystemName, "Wrist Target", "Wrist Target", currentTarget);
      }
      return currentTarget;
    } else {
      // Wrist is not calibrated.  Assume we are at max angle in keepout region to engage all interlocks,
      // since we really don't know where the wrist is at.
      return WristConstants.max;
    }
  }

  /**
	 * returns whether encoder is calibrated or not
	 * @return true if encoder is calibrated and working, false if encoder broke
	 */
	public boolean isEncoderCalibrated() {
		return wristCalibrated;
  }

  /**
   * Writes information about the subsystem to the filelog
   * @param logWhenDisabled true will log when disabled, false will discard the string
   */
  public void updateWristLog(boolean logWhenDisabled) {
    log.writeLog(logWhenDisabled, subsystemName, "Update Variables",
        "Volts", wristMotor.getBusVoltage(), "Amps", wristMotor.getOutputCurrent(),
        "WristCalZero", wristCalZero,
        "Enc Raw", getWristEncoderTicksRaw(), "Wrist Angle", getWristAngle(), "Wrist Target", getCurrentWristTarget(),
        "Upper Limit", getWristUpperLimit(), "Lower Limit", getWristLowerLimit()
        );
  }

  /**
   * Turns file logging on every scheduler cycle (~20ms) or every 10 cycles (~0.2 sec)
   * @param enabled true = every cycle, false = every 10 cycles
   */ 
  @Override
  public void enableFastLogging(boolean enabled) {
    fastLogging = enabled;
  }
  
  @Override
  public void periodic() {

    if (fastLogging || log.isMyLogRotation(logRotationKey)) {
      SmartDashboard.putBoolean(buildString(subsystemName, " calibrated"), wristCalibrated);
      SmartDashboard.putNumber(buildString(subsystemName, " Angle"), getWristAngle());
      SmartDashboard.putNumber(buildString(subsystemName, " enc raw"), getWristEncoderTicksRaw());
			SmartDashboard.putBoolean(buildString(subsystemName, " Lower Limit"), getWristLowerLimit());
      SmartDashboard.putBoolean(buildString(subsystemName, " Upper Limit"), getWristUpperLimit());
      SmartDashboard.putNumber(buildString(subsystemName, " target angle"), getCurrentWristTarget());
      SmartDashboard.putNumber(buildString(subsystemName, " voltage"), wristMotor.getBusVoltage());
    }
    
    // Checks if the wrist is not calibrated and automatically calibrates it once the limit switch is pressed
    // If the wrist isn't calibrated at the start of the match, does that mean we can't control the wrist at all?
    if (!wristCalibrated) {
      if (getWristUpperLimit()) {
        calibrateWristEnc(WristConstants.max);
        updateWristLog(true);
      }
      if (getWristLowerLimit()) {
        calibrateWristEnc(WristConstants.min);
        updateWristLog(true);
      }
    }
    
    // Un-calibrates the wrist if the angle is outside of bounds
    // TODO change low back to - 10.0
    if (getWristAngle() > WristConstants.max + 5.0 || getWristAngle() < WristConstants.min - 15.0) {
      setWristUncalibrated();
      updateWristLog(true);
    }

    if (fastLogging || log.isMyLogRotation(logRotationKey)) {
      updateWristLog(false);
    }

    // if (DriverStation.getInstance().isEnabled()) {
 
      

      /* All of the code below should be gotten rid of for the same reason as the elevator stuff. It doesn't speed anything up in competition - 
      the codriver still has to recognize that the encoders are broken and the wrist is stalled. This is just more code to run in periodic() */
      
      // TO DO: Work on the safety code below.  It tends to trigger if the wrist bounces.


      // Following code checks whether the encoder is incrementing in the same direction as the 
      // motor is moving and changes control modes based on state of encoder
      /*
			currEnc = getWristEncoderTicks();
      if (wristMotor.getMotorOutputVoltage() > 5) {
        if (posMoveCount == 0) {
          encSnapShot = getWristEncoderTicks();
        }
        negMoveCount = 0;
        posMoveCount++;
        if (posMoveCount > 3) {
          if ((currEnc - encSnapShot) < 5) {
            setDefaultCommand(new WristWithXBox());
            Robot.robotPrefs.setWristUncalibrated();
            updateWristLog();
          }
          posMoveCount = 0;
        }
      }
      if (wristMotor.getMotorOutputVoltage() < -5) {
        if (negMoveCount == 0) {
          encSnapShot = getWristEncoderTicks();
        }
        posMoveCount = 0;
        negMoveCount++;
        if (negMoveCount > 3) {
          if ((currEnc - encSnapShot) > -5) {
            // If something is broken, it's just as easy for the codriver to press the 
            // joystick button in before moving it. There's no time savings by having
            // this in periodic.
            setDefaultCommand(new WristWithXBox()); 
            Robot.robotPrefs.setWristUncalibrated();
            updateWristLog();
          }
          negMoveCount = 0;
        }
      }
      */
    // }
  }
 
}