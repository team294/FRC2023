/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.Ports;
import frc.robot.utilities.ElevatorProfileGenerator;
import frc.robot.utilities.FileLog;
import frc.robot.utilities.Loggable;
import frc.robot.utilities.Wait;

/**
 * Add your docs here.
 */
public class Elevator extends SubsystemBase implements Loggable{
	// Put methods for controlling this subsystem
	// here. Call these from Commands.
	private final FileLog log;
	private final int logRotationKey;         // key for the logging cycle for this subsystem
	private boolean fastLogging;
	private final String subsystemName;
	private final WPI_TalonFX elevatorMotor;
	private TalonFXSensorCollection elevatorLimits;

	private ElevatorProfileGenerator elevatorProfile;

	private boolean elevCalibrated = false; // true is encoder is working and calibrated, false is not calibrated
	private boolean elevPosControl = false; // true is in position control mode, false is manual motor control (percent output)

	// TODO Calibrate
	private double rampRate = 0.3;
	private double kP = 0.5;
	private double kI = 0;
	private double kD = 0;
	private double kFF = 0;
	private int kIz = 0;
	private double kMaxOutput = 1.0; // up max output, was 0.8
	private double kMinOutput = -1.0; // down max output, was -0.6

	public double elevatorBottomToFloor; //distance of elevator 0 value from the ground (Not sure we want this because of diagonal elevator)
	// public double elevatorWristSafeStow; 	 // highest elevator position (from ground) where wrist can be stowed (Not sure we need this)

	public Elevator(FileLog log) {
		this.log = log;
		fastLogging = false;
		logRotationKey = log.allocateLogRotation();     // Get log rotation for this subsystem
		subsystemName = "Elevator";

		// Set up motor
		elevatorMotor = new WPI_TalonFX(Ports.CANElevatorMotor);
		elevatorMotor.setInverted(false);
		elevatorMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		elevatorMotor.setSensorPhase(true);         // Flip direction of sensor reading
		elevatorMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
		elevatorMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);

		elevatorLimits = elevatorMotor.getSensorCollection();
		checkAndZeroElevatorEnc();

		elevatorMotor.config_kP(0, kP);
		elevatorMotor.config_kI(0, kI);
		elevatorMotor.config_kD(0, kD);
		elevatorMotor.config_kF(0, kFF);
		elevatorMotor.config_IntegralZone(0, kIz);
		elevatorMotor.configClosedloopRamp(rampRate);
		elevatorMotor.configPeakOutputForward(kMaxOutput);
		elevatorMotor.configPeakOutputReverse(kMinOutput);

		elevatorMotor.clearStickyFaults();
		elevatorMotor.setNeutralMode(NeutralMode.Brake);

		// Wait 0.25 seconds before checking the limit switch or encoder ticks.  The reason is that zeroing the encoder (above)
		// or setting the limit switch type (above) can be delayed up to 50ms for a round trip
		// from the Rio to the Talon and back to the Rio.  So, reading position could give the wrong value if
		// we don't wait (random weird behavior).
		// DO NOT GET RID OF THIS WITHOUT TALKING TO DON OR ROB.
		Wait.waitTime(250);

		// start the elevator in manual mode unless it is properly zeroed
		elevCalibrated = (getElevatorLowerLimit() && getElevatorEncTicks() == 0);

		// create elevator motion profile object
		elevatorProfile = new ElevatorProfileGenerator(this, log);	

		// ensure the elevator starts in manual mode
		stopElevator();
	}

	/**
	 * Returns the name of the subsystem
	 */
	public String getName() {
		return subsystemName;
	}

	/**
	 * Sets elevator to manual control mode with the specified percent output voltage.
	 * @param percentOutput between -1.0 (down) and 1.0 (up)
	 */
	public void setElevatorMotorPercentOutput(double percentOutput) {
		elevatorMotor.set(ControlMode.PercentOutput, percentOutput);
		elevPosControl = false;
	}

	/**
	 * Sets target position for elevator, using motion profile movement.
	 * This only works when encoder is working and elevator is calibrated and the wrist is not interlocked.
	 * @param pos in inches from the floor.
	*/
	public void setProfileTarget(double pos, Wrist wrist) {
		if (elevCalibrated &&										// Elevator must be calibrated
		// Wrist and elevator don't affect eachother 
			  /*wrist.getWristAngle() < WristConstants.stowed &&  	// Wrist must not be stowed
			  wrist.getCurrentWristTarget() < WristConstants.stowed && // Wrist must not be moving to stow
			  ( wrist.getWristAngle() >= WristConstants.wristMinWhenElevatorLow &&	// wrist must be at least wristMinWhenElevatorLow
				wrist.getCurrentWristTarget() >= WristConstants.wristMinWhenElevatorLow ||*/ //Wrist must not be moving to wristMinWhenElevatorLow
				pos >= ElevatorConstants.stowed - 5						// Elevator is not going below stowed position
				/*&& wrist.getWristAngle() >= WristConstants.wristDown - 3.0 &&	     // wrist must be at least wristDown
				wrist.getCurrentWristTarget() >= WristConstants.wristDown - 3.0 */// Wrist must not be moving below wristDown
		 ) {
			elevPosControl = true;
			elevatorProfile.setProfileTarget(pos);
			log.writeLog(false, subsystemName, "setProfileTarget", "Target", pos, "Allowed,Yes,Wrist Angle",
			   wrist.getWristAngle(), "Wrist Target", wrist.getCurrentWristTarget());
		} else {
			log.writeLog(false, subsystemName, "setProfileTarget", "Target", pos, "Allowed,No,Wrist Angle",
 			  wrist.getWristAngle(), "Wrist Target", wrist.getCurrentWristTarget());
		}
	}

	/**
	 * Set elevator position using the Talon PID.
	 * works same with falcon?
	 * This only works when encoder is working and elevator is calibrated and the wrist is not interlocked.
	 * @param inches target height in inches off the floor
	 *  Should we make it from start of elevator instead of from the floor?
	 */
	public void setElevatorPos(Wrist wrist, double inches) {
		if (elevCalibrated &&										// Elevator must be calibrated
		// Wrist and elevator don't affect eachother 
			 /* wrist.getWristAngle() < WristConstants.stowed &&  	// Wrist must not be stowed
			  wrist.getCurrentWristTarget() < WristConstants.stowed && // Wrist must not be moving to stow
			  ( wrist.getWristAngle() >= WristConstants.straight - 5.0 &&	// wrist must be at least horizontal
				wrist.getCurrentWristTarget() >= WristConstants.straight - 5.0 || // Wrist must not be moving below horizantal */
				inches >= ElevatorConstants.stowed //&&						// Elevator is not going below groundCargo position
				// wrist.getWristAngle() >= WristConstants.wristDown - 3.0 &&	     // wrist must be at least wristDown
				// wrist.getCurrentWristTarget() >= WristConstants.wristDown - 3.0 ) // Wrist must not be moving below wristDown
		 ) {
			elevatorMotor.set(ControlMode.Position, inchesToEncoderTicks(inches - elevatorBottomToFloor));
			elevPosControl = true;
			log.writeLog(false, subsystemName, "Position set", "Target", inches, "Allowed,Yes,Wrist Angle",
			   wrist.getWristAngle(), "Wrist Target", wrist.getCurrentWristTarget());
		} else {
			log.writeLog(false, subsystemName, "Position set", "Target", inches, "Allowed,No,Wrist Angle",
 			  wrist.getWristAngle(), "Wrist Target", wrist.getCurrentWristTarget());
		}
	}

	/**
	 * 2023 - Might not want inches from floor this year because elevator is diagonal
	 * return stowed position if not calibrated?
	 * Returns the height that elevator is trying to move to in inches from the floor.
	 * Returns hatchHigh if the elevator is in manual mode (not calibrated), in order to engage interlocks.
	 * <p><b>NOTE:</b> This is the target height, not the current height.
	 * If the elevator is in manual control mode, returns the actual elevator position.
	 * @return desired inches of elevator height
	 */
	public double getCurrentElevatorTarget() {
		if (elevCalibrated) {
			if (elevPosControl) {
				if (elevatorMotor.getControlMode() == ControlMode.Position) {
					// Closed loop control using Talon PID
					// same with falcon?
					return encoderTicksToInches(elevatorMotor.getClosedLoopTarget(0)) + elevatorBottomToFloor;
				} else {
					// Motion profile control
					return elevatorProfile.getFinalPosition();
				}
			} else {
				// Manual control mode
				return getElevatorPos();
			}
		} else {
			// Elevator not calibrated
			return ElevatorConstants.stowed; // Not sure about this
		}
	}

	/**
	 * @return Current elevator position, in inches from floor.  
	 * if the elevator is in manual mode (not calibrated) return stowed?
	 * 
	 */
	public double getElevatorPos() {
		if (elevCalibrated) {
			return encoderTicksToInches(getElevatorEncTicks()) + elevatorBottomToFloor;
		} else {
			return ElevatorConstants.stowed;   //This could be a problem on next time it is enable it goes to hatchHigh
		}
	}

	/**
	 * @return Current elevator velocity in in/s, + equals up, - equals down
	 */
	public double getElevatorVelocity() {
		return encoderTicksToInches(elevatorMotor.getSelectedSensorVelocity(0) * 10.0);
	}

	/**
	 * stops elevator motors
	 */
	public void stopElevator() {
		setElevatorMotorPercentOutput(0.0);
	}

	/**
	 * only zeros elevator encoder when it is at the zero position (lower limit)
	 */
	public void checkAndZeroElevatorEnc() {
		if (getElevatorLowerLimit()) {
			stopElevator();			// Make sure Talon PID loop or motion profile won't move the robot to the last set position when we reset the enocder position
			elevatorMotor.setSelectedSensorPosition(0, 0, 0);
			elevCalibrated = true;
			log.writeLog(false, subsystemName, "Calibrate and Zero Encoder", "checkAndZeroElevatorEnc");
		}
	}

	/**
	 * Returns if the encoder is calibrated and working
	 * @return true = working, false = not working
	 */
	public boolean encoderCalibrated() {
		return elevCalibrated;
	}

	/**
	 * @return raw encoder ticks (based on encoder zero being at zero position)
	 */
	public double getElevatorEncTicks() {
		return elevatorMotor.getSelectedSensorPosition(0);
	}

	/**
	 * @param encoderTicks in enocder Ticks
	 * @return parameter encoder ticks converted to equivalent inches
	 */
	public double encoderTicksToInches(double encoderTicks) {
		return (encoderTicks * ElevatorConstants.kElevEncoderInchesPerTick);
	}

	/**
	 * @param inches in inches
	 * @return parameter inches converted to equivalent encoder ticks
	 */
	public double inchesToEncoderTicks(double inches) {
		return (inches / ElevatorConstants.kElevEncoderInchesPerTick);
	}

	/**
	 * reads whether the elevator is at the upper limit
	 */
	public boolean getElevatorUpperLimit() {
		return elevatorLimits.isFwdLimitSwitchClosed() == 1;
	}

	/**
	 * reads whether the elevator is at the lower limit
	 */
	public boolean getElevatorLowerLimit() {
		return elevatorLimits.isRevLimitSwitchClosed() == 1;
	}

	/**
    * Writes information about the subsystem to the filelog
    * @param logWhenDisabled true will log when disabled, false will discard the string
    */
	public void updateElevatorLog(boolean logWhenDisabled) {
		log.writeLog(logWhenDisabled, subsystemName, "Update Variables",
				"Volts", elevatorMotor.getMotorOutputVoltage(), "Amps", elevatorMotor.getStatorCurrent(),
				"Enc Ticks", getElevatorEncTicks(), "Enc Inches", getElevatorPos(), 
				"Elev Target", getCurrentElevatorTarget(), "Elev Vel", getElevatorVelocity(),
				"Upper Limit", getElevatorUpperLimit(), "Lower Limit", getElevatorLowerLimit(),
				"Elev Mode", elevCalibrated);
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
		
		if (log.isMyLogRotation(logRotationKey)) {
			SmartDashboard.putBoolean("Elev Calibrated", elevCalibrated);
			SmartDashboard.putBoolean("Elev Mode", elevPosControl);
			SmartDashboard.putNumber("Elev Pos", getElevatorPos());
			SmartDashboard.putNumber("Elev Target", getCurrentElevatorTarget());
			SmartDashboard.putNumber("Elev Ticks", getElevatorEncTicks());
			// SmartDashboard.putNumber("Elev Enc Tick", getElevatorEncTicks());
			SmartDashboard.putBoolean("Elev Lower Limit", getElevatorLowerLimit());
			SmartDashboard.putBoolean("Elev Upper Limit", getElevatorUpperLimit());
		}

		if (fastLogging || log.isMyLogRotation(logRotationKey)) {
			updateElevatorLog(false);
			elevatorProfile.updateElevatorProfileLog(false);
		}

		// Sets elevator motors to percent power required as determined by motion profile.
		// Only set percent power IF the motion profile is enabled.
		// Note:  If we are using our motion profile control loop, then set the power directly using elevatorMotor1.set().
		// Do not call setElevatorMotorPercentOutput(), since that will change the elevPosControl to false (manual control).
		if (elevPosControl && elevatorMotor.getControlMode() != ControlMode.Position) {
			elevatorMotor.set(ControlMode.PercentOutput, elevatorProfile.trackProfilePeriodic());  
		}

		// Autocalibrate in the encoder is OK and the elevator is at the lower limit switch
		if ((!elevCalibrated || Math.abs(getElevatorEncTicks()) > 600) && getElevatorLowerLimit()) {
			setDefaultCommand(null);
			elevCalibrated = true;
			stopElevator();
			elevatorMotor.setSelectedSensorPosition(0, 0, 0);
			log.writeLog(false, subsystemName, "Calibrate and Zero Encoder", "periodic");
		}
		
	}
}