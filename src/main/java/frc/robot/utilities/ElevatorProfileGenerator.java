package frc.robot.utilities;

import frc.robot.Robot;
import frc.robot.subsystems.Elevator;

import java.io.File;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorProfileGenerator {

	Elevator elevator;
	FileLog log;
	private boolean profileEnabled = false;

	private double currentMPDistance; // Distance that should have been travelled in current motion profile (always positive)
	private double targetMPDistance; // Total distance to be travelled in current motion profile (always positive)

	private double initialPosition; // Initial position in inches from the floor
	private double finalPosition; // Final position in inches from the floor

	private double maxVelocity = 50;
	private double currentMPVelocity;

	private double maxAcceleration = 100;
	private double stoppingAcceleration = .5 * maxAcceleration;
	private double currentMPAcceleration;
	private boolean approachingTarget = false;		// true = decelerating towards target;  false = not close enough to start decelerating

	private double dt; // delta T (time)

	private double directionSign; // +1 if finalPosition > InitialPosition, -1 if not

	private long startTime, lastTime;

	private double prevError, error, intError;

	double percentPowerFF = 0;
	double percentPowerFB = 0;

	private double kFF = 0.14;  // calibrated to 0.1, 0.12
	private double kVu = 0.0139;  // Should be around 0.015, calibrated to 0.0139
	private double kAu = 0.002;   // Should be around 0.002
	private double kPu = 0.15;  // was 0.15
	private double kIu = 0;
	private double kDu = 0;
	private double kVd = 0.0125;   // Should be around 0.012, calibrated to 0.0125
	private double kAd = 0.002;   // Should be around 0.002
	private double kPd = 0.15;	// was 0.05
	private double kId = 0;
	private double kDd = 0;
	
	/**
	 * Creates a new profile generator but keeps it in disabled mode
	 */
	public ElevatorProfileGenerator(Elevator elevator, FileLog log) {
		disableProfileControl();
		this.log = log;
		this.elevator = elevator;
	}

	/**
	 * disables motion profile's control of motors
	 */
	public void disableProfileControl() {
		profileEnabled = false;
	}

	/**
	 * Sets target position for elevator, using motion profile movement.
	 * This enables the profiler to take control of the elevator, so only call it if the
	 * encoder is working, the elevator is calibrated, and there are no physical obstructions
	 * (check interlocks before calling).
	 * Enables motion profile's control of motors
	 * @param pos in inches from the floor.
	*/
	public void setProfileTarget(double pos) {
		profileEnabled = true;
		finalPosition = pos;
		initialPosition = elevator.getElevatorPos();
		intError = 0;			// Clear integrated error
		prevError = 0;			// Clear previous error
		approachingTarget = false;

		// Save starting time
		startTime = System.currentTimeMillis();
		lastTime = startTime;

		SmartDashboard.putNumber("ElevatorInitPos", initialPosition);
		SmartDashboard.putNumber("ElevatorTarget", finalPosition);

		currentMPDistance = 0;
		targetMPDistance = Math.abs(finalPosition - initialPosition);

		directionSign = Math.signum(finalPosition - initialPosition);

		/* TODO uncomment if we decide to have different velocities/accelerations for up vs down
		if(directionSign == 1) {

		} else {

		} */

		// Seed the profile with the current velocity, in case the elevator is already moving
		currentMPVelocity = elevator.getElevatorVelocity() * directionSign;

		log.writeLog(false, "ElevatorProfile", "New Profile", "Init pos", initialPosition , "Final pos", finalPosition);
	}

	/**
	 * Call this method once per scheduler cycle. This method calculates the
	 * distance that the robot should have traveled at this point in time, per the
	 * motion profile. Also calculates velocity in in/s
	 */
	public void updateProfileCalcs() {
		if (currentMPDistance < targetMPDistance) { 
			// does not continue calculating after we should have reached our target (within a quarter inch)
			long currentTime = System.currentTimeMillis();
			dt = ((double) (currentTime - lastTime)) / 1000.0;
			lastTime = currentTime;

			double stoppingDistance = 0.5 * currentMPVelocity * currentMPVelocity / stoppingAcceleration;

			// calculating target acceleration
			if ( (currentMPVelocity > 0) &&
			       (approachingTarget || (targetMPDistance - currentMPDistance) < stoppingDistance) ) {
				approachingTarget = true;
				currentMPAcceleration = -0.5 * currentMPVelocity * currentMPVelocity / (targetMPDistance - currentMPDistance);
			}
			else if (currentMPVelocity < maxVelocity) {
				currentMPAcceleration = maxAcceleration;
			}
			else {
				currentMPAcceleration = 0;
			}

			// calculating target velocity
			currentMPVelocity = currentMPVelocity + currentMPAcceleration * dt;
			if (currentMPVelocity > maxVelocity) {
				currentMPVelocity = maxVelocity;
			}

			// calculating the distance the elevator should have travelled
			currentMPDistance = currentMPDistance + currentMPVelocity * dt;
			if (currentMPDistance > targetMPDistance) {
				currentMPDistance = targetMPDistance;
			}

		} else { // do not change the theoretical distance once it has reached the target
			currentMPDistance = targetMPDistance;
			currentMPVelocity = 0;
			currentMPAcceleration = 0;
		}
	}


	/**
	 * Code to make the elevator follow the MotionProfile, should be called exactly once per scheduler cycle.
	 * ONLY follow the MotionProfile if elevPosControl is true, else we should be in manual mode so do nothing.
	 * @return percent power to set the elevator motors to based on calculations
	 */
	public double trackProfilePeriodic() {
		if(profileEnabled) {
			updateProfileCalcs();
			error = getCurrentPosition() - elevator.getElevatorPos();
			intError = intError + error * dt;

			if (directionSign == 1) {
				percentPowerFF = kFF + kVu*currentMPVelocity*directionSign + kAu*currentMPAcceleration*directionSign;
				percentPowerFB = kPu * error + ((error - prevError) * kDu) + (kIu * intError);
			} else if(directionSign == -1) {
				percentPowerFF = kFF + kVd*currentMPVelocity*directionSign + kAd*currentMPAcceleration*directionSign;
				percentPowerFB = kPd * error + ((error - prevError) * kDd) + (kId * intError);
			} 
			prevError = error;

			// Cap feedback power to prevent jerking the elevator
			percentPowerFB = (percentPowerFB>0.2) ? 0.2 : percentPowerFB;
			percentPowerFB = (percentPowerFB<-0.2) ? -0.2 : percentPowerFB;

			if (log.getLogLevel()<=1 || currentMPVelocity>0 || Math.abs(percentPowerFB)>0.1) {
				updateElevatorProfileLog(false);
			}

			return percentPowerFF + percentPowerFB;
		} else {
			return 0.0;
		}
	}

	/**
    * Writes information about the elevator profile to the filelog
    * @param logWhenDisabled true will log when disabled, false will discard the string
    */
	public void updateElevatorProfileLog(boolean logWhenDisabled) {
		log.writeLog(logWhenDisabled, "ElevatorProfile", "updateCalc",
		        "MP Pos", getCurrentPosition(), "ActualPos",
				elevator.getElevatorPos(), "TargetPos",
				finalPosition, "Time since start", getTimeSinceProfileStart(), "dt", dt,
				"ActualVel", elevator.getElevatorVelocity(),
				"MP Vel,", (currentMPVelocity * directionSign),
				"MP Accel", (currentMPAcceleration * directionSign),
				"PowerFF", percentPowerFF, "PowerFB", percentPowerFB );
	}

	/**
	 * @return Current target position for the robot, in inches
	 */
	public double getCurrentPosition() {
		return currentMPDistance * directionSign + initialPosition;

	}

	/**
	 * @return Current final position of motion profile
	 */
	public double getFinalPosition() {
		return finalPosition;
	}

	/**
	 * @return time in seconds since starting the current profile
	 */
	public double getTimeSinceProfileStart() {
		return ((double) (lastTime - startTime)) / 1000.0;
	}

	/**
	 * @return Current target velocity from profile calculation in in/s
	 */
	public double getCurrentVelocity() {
		return currentMPVelocity * directionSign;
	}
}
