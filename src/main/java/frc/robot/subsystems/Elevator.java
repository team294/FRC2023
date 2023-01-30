/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Constants.Ports;
import frc.robot.Constants.WristConstants;
import frc.robot.Constants.ElevatorConstants;

// import frc.robot.commands.ElevatorWithXBox;
import frc.robot.utilities.ElevatorProfileGenerator;
import frc.robot.utilities.FileLog;
import frc.robot.utilities.RobotPreferences;
import frc.robot.utilities.Wait;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;

/**
 * Add your docs here.
 */
public class Elevator extends SubsystemBase {
	// Put methods for controlling this subsystem
	// here. Call these from Commands.
	FileLog log;
	private int logRotationKey;         // key for the logging cycle for this subsystem
	private WPI_TalonFX elevatorMotor1;
	private WPI_TalonFX elevatorMotor2;
	private TalonFXSensorCollection elevatorLimits;

	private ElevatorProfileGenerator elevatorProfile;

	// private int posMoveCount = 0; // increments every cycle the elevator moves up
	// private int negMoveCount = 0; // increments every cycle the elevator moves down
	// private double currEnc = 0.0; // current recorded encoder value
	// private double encSnapShot = 0.0; // snapshot of encoder value used to make sure encoder is working
	private int motorFaultCount = 0; // increments every cycle the motor detects an issue
	private boolean elevEncOK = true; // true is encoder working, false is encoder broken
	private boolean elevCalibrated = false; // true is encoder is working and calibrated, false is not calibrated
	private boolean elevPosControl = false; // true is in position control mode, false is manual motor control (percent output)

	private double rampRate = 0.3;
	private double kP = 0.5;
	private double kI = 0;
	private double kD = 0;
	private double kFF = 0;
	private int kIz = 0;
	private double kMaxOutput = 1.0; // up max output, was 0.8
	private double kMinOutput = -1.0; // down max output, was -0.6

	public double elevatorGearCircumference; //circumference of the gear driving the elevator in inches
	public double elevatorBottomToFloor; //distance of elevator 0 value from the ground
	public double elevatorWristSafeStow; 	 // highest elevator position (from ground) where wrist can be stowed

	public Elevator(FileLog log) {
		this.log = log;
		logRotationKey = log.allocateLogRotation();     // Get log rotation for this subsystem
		elevatorMotor1 = new WPI_TalonFX(Ports.CANElevatorMotor1);
		elevatorMotor2 = new WPI_TalonFX(Ports.CANElevatorMotor2);
		elevatorMotor2.follow(elevatorMotor1);
		elevatorMotor1.setInverted(false);
		elevatorMotor2.setInverted(true);
		elevatorMotor1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		elevatorMotor1.setSensorPhase(true);         // Flip direction of sensor reading
		elevatorMotor1.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
		elevatorMotor1.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);

		elevatorLimits = elevatorMotor1.getSensorCollection();
		checkAndZeroElevatorEnc();

		elevatorMotor1.config_kP(0, kP);
		elevatorMotor1.config_kI(0, kI);
		elevatorMotor1.config_kD(0, kD);
		elevatorMotor1.config_kF(0, kFF);
		elevatorMotor1.config_IntegralZone(0, kIz);
		elevatorMotor1.configClosedloopRamp(rampRate);
		elevatorMotor1.configPeakOutputForward(kMaxOutput);
		elevatorMotor1.configPeakOutputReverse(kMinOutput);

		elevatorMotor1.clearStickyFaults();
		elevatorMotor2.clearStickyFaults();
		elevatorMotor1.setNeutralMode(NeutralMode.Brake);
		elevatorMotor2.setNeutralMode(NeutralMode.Brake);

		// if (Robot.robotPrefs.neoDrivetrain) {
			elevatorMotor2.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 5);
		// }

		// Wait 0.25 seconds before checking the limit switch or encoder ticks.  The reason is that zeroing the encoder (above)
		// or setting the limit swtich type (above) can be delayed up to 50ms for a round trip
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
	 * Sets elevator to manual control mode with the specified percent output voltage.
	 * @param percentOutput between -1.0 (down) and 1.0 (up)
	 */
	public void setElevatorMotorPercentOutput(double percentOutput) {
		elevatorMotor1.set(ControlMode.PercentOutput, percentOutput);
		elevPosControl = false;
	}

	/**
	 * Sets target position for elevator, using motion profile movement.
	 * This only works when encoder is working and elevator is calibrated and the wrist is not interlocked.
	 * @param pos in inches from the floor.
	*/
	public void setProfileTarget(double pos, Wrist wrist) {
		if (elevEncOK && elevCalibrated &&										// Elevator must be calibrated
			  wrist.getWristAngle() < WristConstants.wristKeepOut &&  	// Wrist must not be stowed
			  wrist.getCurrentWristTarget() < WristConstants.wristKeepOut && // Wrist must not be moving to stow
			  ( wrist.getWristAngle() >= WristConstants.wristLowerCrashWhenElevatorLow &&	// wrist must be at least wristLowerCrashWhenElevatorLow
				wrist.getCurrentWristTarget() >= WristConstants.wristLowerCrashWhenElevatorLow ||
				pos >= ElevatorConstants.groundCargo &&						// Elevator is not going below groundCargo position
				wrist.getWristAngle() >= WristConstants.wristDown - 3.0 &&	     // wrist must be at least wristDown
				wrist.getCurrentWristTarget() >= WristConstants.wristDown - 3.0 )
		 ) {
			elevPosControl = true;
			elevatorProfile.setProfileTarget(pos);
			log.writeLog(false,"Elevator", "setProfileTarget", "Target", pos, "Allowed,Yes,Wrist Angle",
			   wrist.getWristAngle(), "Wrist Target", wrist.getCurrentWristTarget());
		} else {
			log.writeLog(false, "Elevator", "setProfileTarget", "Target", pos, "Allowed,No,Wrist Angle",
 			  wrist.getWristAngle(), "Wrist Target", wrist.getCurrentWristTarget());
		}
	}

	/**
	 * Set elevator position using the Talon PID.  This
	 * only works when encoder is working and elevator is calibrated and the wrist is not interlocked.
	 * @param inches target height in inches off the floor
	 */
	public void setElevatorPos(Wrist wrist, double inches) {
		if (elevEncOK && elevCalibrated &&										// Elevator must be calibrated
			  wrist.getWristAngle() < WristConstants.wristKeepOut &&  	// Wrist must not be stowed
			  wrist.getCurrentWristTarget() < WristConstants.wristKeepOut && // Wrist must not be moving to stow
			  ( wrist.getWristAngle() >= WristConstants.wristStraight - 5.0 &&	// wrist must be at least horizontal
				wrist.getCurrentWristTarget() >= WristConstants.wristStraight - 5.0 ||
				inches >= ElevatorConstants.groundCargo &&						// Elevator is not going below groundCargo position
				wrist.getWristAngle() >= WristConstants.wristDown - 3.0 &&	     // wrist must be at least wristDown
				wrist.getCurrentWristTarget() >= WristConstants.wristDown - 3.0 )
		 ) {
			elevatorMotor1.set(ControlMode.Position, inchesToEncoderTicks(inches - elevatorBottomToFloor));
			elevPosControl = true;
			log.writeLog(false, "Elevator", "Position set", "Target", inches, "Allowed,Yes,Wrist Angle",
			   wrist.getWristAngle(), "Wrist Target", wrist.getCurrentWristTarget());
		} else {
			log.writeLog(false, "Elevator", "Position set", "Target", inches, "Allowed,No,Wrist Angle",
 			  wrist.getWristAngle(), "Wrist Target", wrist.getCurrentWristTarget());
		}
	}

	/**
	 * Returns the height that elevator is trying to move to in inches from the floor.
	 * Returns hatchHigh if the elevator is in manual mode (not calibrated), in order to engage interlocks.
	 * <p><b>NOTE:</b> This is the target height, not the current height.
	 * If the elevator is in manual control mode, returns the actual elevator position.
	 * @return desired inches of elevator height
	 */
	public double getCurrentElevatorTarget() {
		if (elevCalibrated) {
			if (elevPosControl) {
				if (elevatorMotor1.getControlMode() == ControlMode.Position) {
					// Closed loop control using Talon PID
					return encoderTicksToInches(elevatorMotor1.getClosedLoopTarget(0)) + elevatorBottomToFloor;
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
			return ElevatorConstants.hatchHigh;
		}
	}

	/**
	 * @return Current elevator position, in inches from floor.  Returns hatchHigh
	 * if the elevator is in manual mode (not calibrated), in order to engage interlocks.
	 */
	public double getElevatorPos() {
		if (elevCalibrated) {
			return encoderTicksToInches(getElevatorEncTicks()) + elevatorBottomToFloor;
		} else {
			return ElevatorConstants.hatchHigh;   //This could be a problem on next time it is enable it goes to hatchHigh
		}
	}

	/**
	 * @return Current elevator velocity in in/s, + equals up, - equals down
	 */
	public double getElevatorVelocity() {
		return encoderTicksToInches(elevatorMotor1.getSelectedSensorVelocity(0) * 10.0);
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
			elevatorMotor1.setSelectedSensorPosition(0, 0, 0);
			elevCalibrated = true;
			log.writeLog(false, "Elevator", "Calibrate and Zero Encoder", "checkAndZeroElevatorEnc");
		}
	}

	/**
	 * Returns if the encoder is calibrated and working
	 * @return true = working, false = not working
	 */
	public boolean encoderCalibrated() {
		return elevEncOK && elevCalibrated;
	}

	/**
	 * @return raw encoder ticks (based on encoder zero being at zero position)
	 */
	public double getElevatorEncTicks() {
		return elevatorMotor1.getSelectedSensorPosition(0);
	}

	/**
	 * @param encoderTicks in enocder Ticks
	 * @return parameter encoder ticks converted to equivalent inches
	 */
	public double encoderTicksToInches(double encoderTicks) {
		return (encoderTicks / WristConstants.encoderTicksPerRevolution) * (elevatorGearCircumference * 2);
	}

	/**
	 * @param inches in inches
	 * @return parameter inches converted to equivalent encoder ticks
	 */
	public double inchesToEncoderTicks(double inches) {
		return (inches / (elevatorGearCircumference * 2)) * ElevatorConstants.encoderTicksPerRevolution;
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
	 * Checks elevator motor currents, records sticky faults if a motor is faulty for more than 5 cycles
	 */
	public void verifyMotors() {
		double amps1 = elevatorMotor1.getStatorCurrent();
		double amps2 = elevatorMotor2.getStatorCurrent();

		if(motorFaultCount >= 5) {
			RobotPreferences.recordStickyFaults("Elevator", log);
			motorFaultCount = 0;
		}

		if(amps1 > 8 && amps2 < 3) {
			motorFaultCount++;
		}
		else if(amps2 > 8 && amps1 < 3) {
			motorFaultCount++;
		}
		else {
			motorFaultCount = 0;
		}
	}

	/**
    * Writes information about the subsystem to the filelog
    * @param logWhenDisabled true will log when disabled, false will discard the string
    */
	public void updateElevatorLog(boolean logWhenDisabled) {
		log.writeLog(logWhenDisabled, "Elevator", "Update Variables",
				"Volts1," + elevatorMotor1.getMotorOutputVoltage(), "Volts2", elevatorMotor2.getMotorOutputVoltage(),
				"Amps1", elevatorMotor1.getStatorCurrent(), "Amps2", elevatorMotor2.getStatorCurrent() + 
				"Enc Ticks", getElevatorEncTicks(), "Enc Inches", getElevatorPos(), 
				"Elev Target", getCurrentElevatorTarget(), "Elev Vel", getElevatorVelocity(),
				"Upper Limit", getElevatorUpperLimit(), "Lower Limit", getElevatorLowerLimit(),
				"Enc OK", elevEncOK, "Elev Mode", elevCalibrated);
	}

	// @Override
	// public void initDefaultCommand() {
	// 	// Set the default command for a subsystem here.
	// 	if (!elevCalibrated) {
	// 		setDefaultCommand(new ElevatorWithXBox());
	// 	}
	// }

	
	@Override
	public void periodic() {
		
		// Can some of these be eliminated by competition?

		if (log.getLogRotation() == logRotationKey) {
			SmartDashboard.putBoolean("Elev encOK", elevEncOK);
			SmartDashboard.putBoolean("Elev Calibrated", elevCalibrated);
			SmartDashboard.putBoolean("Elev Mode", elevPosControl);
			// SmartDashboard.putNumber("EncSnap", encSnapShot);
			// SmartDashboard.putNumber("Enc Now", currEnc);
			SmartDashboard.putNumber("Elev Pos", getElevatorPos());
			SmartDashboard.putNumber("Elev Target", getCurrentElevatorTarget());
			SmartDashboard.putNumber("Elev Ticks", getElevatorEncTicks());
			// SmartDashboard.putNumber("Enc Tick", getElevatorEncTicks());
			SmartDashboard.putBoolean("Elev Lower Limit", getElevatorLowerLimit());
			SmartDashboard.putBoolean("Elev Upper Limit", getElevatorUpperLimit());

			updateElevatorLog(false);
			elevatorProfile.updateElevatorProfileLog(false);
		}

		// Sets elevator motors to percent power required as determined by motion profile.
		// Only set percent power IF the motion profile is enabled.
		// Note:  If we are using our motion profile control loop, then set the power directly using elevatorMotor1.set().
		// Do not call setElevatorMotorPercentOutput(), since that will change the elevPosControl to false (manual control).
		if (elevPosControl && elevatorMotor1.getControlMode() != ControlMode.Position) {
			elevatorMotor1.set(ControlMode.PercentOutput, elevatorProfile.trackProfilePeriodic());  
		}

		// Following code changes the frequency of variable logging depending
		// on the set logLevel, Motors are checked every cycle regardless
		if (DriverStation.isEnabled()) {

			verifyMotors(); // What is the concrete use for this?  Move to a pit command, instead of live during match?

			//TODO simplify the encoder check.  If output voltage is greater than 3V (5V?) and encoder does not change more than
			// 3 ticks in 5 cycles, then set elevEncOK = false.  If the encoder is moving, then set elevEncOK = true.
			//  Test and verify that it does not false trigger, but does trigger when the encoder is unplugged.
			// Also verify that it correctly resets the elevEncOK to true if the encoder starts working again.
		
			// Following code checks whether the encoder is incrementing in the same direction as the 
			// motor is moving and changes control modes based on state of encoder

			/* All of the code below should be gotten rid of. It doesn't speed anything up in competition - the codriver still has to recognize that the encoders are broken
			and the elevator is stalled. This is just more code to run in periodic() */
			// TO DO: The code below is causing false triggers that causes the elevator to be uncalibrated.
			/*
			currEnc = getElevatorEncTicks();
			if (elevatorMotor1.getMotorOutputVoltage() > 5) {
				if (posMoveCount == 0) {
					encSnapShot = getElevatorEncTicks();
				}
				negMoveCount = 0;
				posMoveCount++;
				if (posMoveCount > 3) {
					elevEncOK = (currEnc - encSnapShot) > 100;
					if (!elevEncOK) {
						Robot.robotPrefs.recordStickyFaults("Elevator Enc");
						setDefaultCommand(new ElevatorWithXBox()); 
						// We can probably ignore the automatic switch to xbox. By the time the codriver realizes, they can just push the joystick button in anyways.
						elevatorMode = false;
					}
					posMoveCount = 0;
				}
			} else if (elevatorMotor1.getMotorOutputVoltage() < -5) {
				if (negMoveCount == 0) {
					encSnapShot = getElevatorEncTicks();
				}
				posMoveCount = 0;
				negMoveCount++;
				if (negMoveCount > 3) {
					elevEncOK = (currEnc - encSnapShot) < -100;
					if (!elevEncOK) {
						Robot.robotPrefs.recordStickyFaults("Elevator Enc");
						setDefaultCommand(new ElevatorWithXBox());
						// We can probably ignore the automatic switch to xbox. By the time the codriver realizes, they can just push the joystick button in anyways.
						elevatorMode = false;
					}
					negMoveCount = 0;
				}
			}
			*/
		}

		// Autocalibrate in the encoder is OK and the elevator is at the lower limit switch
		if ((!elevCalibrated || Math.abs(getElevatorEncTicks()) > 600) && elevEncOK && getElevatorLowerLimit()) {
			setDefaultCommand(null);
			elevCalibrated = true;
			stopElevator();
			elevatorMotor1.setSelectedSensorPosition(0, 0, 0);
			log.writeLog(false, "Elevator", "Calibrate and Zero Encoder", "periodic");

			// posMoveCount = 0;
			// negMoveCount = 0;
		}
		
	}
}