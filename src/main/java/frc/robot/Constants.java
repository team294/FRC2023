// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import frc.robot.utilities.TrapezoidProfileBCR;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public enum CoordType {
        kRelative,              // Relative to current robot location/facing
        kAbsolute,              // Absolute field coordinates, don't reset robot pose
        kAbsoluteResetPose,     // Absolute field coordinates, reset robot pose always
        kAbsoluteResetPoseTol;  // Absolute field coordinates, reset robot pose if robot is not close to specified position
    }

    /**
     * Options to select driving stopping types.
     */
    public enum StopType {
        kNoStop,
        kCoast,
        kBrake;
    }


    public static final class Ports{
        public static final int CANPneumaticHub = 1;

        public static final int CANDriveFrontLeftMotor = 1;
        public static final int CANDriveFrontRightMotor = 2;
        public static final int CANDriveBackLeftMotor = 3;
        public static final int CANDriveBackRightMotor = 4;

        public static final int CANDriveTurnFrontLeftMotor = 5;
        public static final int CANDriveTurnFrontRightMotor = 6;
        public static final int CANDriveTurnBackLeftMotor = 7;
        public static final int CANDriveTurnBackRightMotor = 8;

        // Note:  Remote sensors accessed by a Talon FX (Falcon 500) must have a CAN ID of 15 or less. See errata
        // in CTRE documentation "Talon FX Remote Filter Device ID Must be 15 or Less" for more details.
        // This applies to the turn encoders, which are used as remote sensors for the turn motors.
        public static final int CANTurnEncoderFrontLeft = 9;
        public static final int CANTurnEncoderFrontRight = 10;
        public static final int CANTurnEncoderBackLeft = 11;
        public static final int CANTurnEncoderBackRight = 12;

        public static final int CANElevatorMotor = 21;      //TODO CHANGE NUMBER TO REAL PORT 
        // public static final int CANElevatorMotor2 = 22;
        public static final int CANWristMotor = 23;         //TODO CHANGE NUMBER TO REAL PORT 

        public static final int CANGrabber = 44;
        public static final int CANManipulator = 45; //TODO CHANGE NUMBER TO REAL PORT 

        // Digital IO ports
        public static final int DIOManipulatorCubeSensor = 5; //TODO PLACE HOLDER SET TO CORRET PORT
        public static final int DIOManipulatorConeSensor = 6; //TODO PLACE HOLDER SET TO CORRECT PORT

        // PWM ports
        public static final int PWMLEDStripTop = 0;         // LED Strip on top of robot

        // I2C ports
        // public static final int I2CcolorSensor = 0x52;       // According to REV docs, color sensor is at 0x52 = 82.  Rob had 39?

        // Pneumatic solenoid ports
        public static final int SolManipulatorFwd = 0;      // TODO set correct channel
        public static final int SolManipulatorRev = 1;      // TODO set correct channel

    }

    public static final class OIConstants {
        public static final int usbXboxController = 0;
        public static final int usbLeftJoystick = 1;
        public static final int usbRightJoystick = 2;
        public static final int usbCoPanel = 3;

        public static final double joystickDeadband = 0.01;
    }

    public static final class RobotDimensions {
        //left to right distance between the drivetrain wheels; should be measured from center to center
        public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.58721;      // CALIBRATED-3 = 0.58721 (based on robot rotating in place).  CAD geometry = 0.57785.
        //front-back distance between the drivetrain wheels; should be measured from center to center
        public static final double DRIVETRAIN_WHEELBASE_METERS = 0.58721;       // CALIBRATED-3 = 0.58721 (based on robot rotating in place).  CAD geometry = 0.57785.

    }

    public static final class SwerveConstants {
        // Encoder calibration to meters travelled or wheel facing degrees
        public static final double kEncoderCPR = 2048.0;                // CALIBRATED-3 = 2048.  Encoder counts per revolution of FalconFX motor pinion gear
        public static final double kDriveGearRatio = (6.75 / 1.0);      // CALIBRATED-3 = 6.75/1.0.  Team364 (MK3i?) = 6.86:1.  Mk4i = 8.14:1 (L1-std gears).  Mk4i = 6.75:1 (L2-fast gears)
        public static final double kTurningGearRatio = (150.0/7.0 / 1.0); // CALIBRATED-3 = 150.0/7.0.  Team364 (MK3i?) = 12.8:1.  Mk4i = 150/7 : 1
        public static final double kWheelDiameterMeters = 0.09712;        // CALIBRATED-3 = 0.09712.  Depends a little on the tread wear!
        public static final double kDriveEncoderMetersPerTick = (kWheelDiameterMeters * Math.PI) / kEncoderCPR / kDriveGearRatio;
        public static final double kTurningEncoderDegreesPerTick = 360.0/kEncoderCPR / kTurningGearRatio;
        
        // Robot calibration for feed-forward and max speeds
        // Max speed is used to keep each motor from maxing out, which preserves ratio between motors 
        // and ensures that the robot travels in the requested direction.  So, use min value of all 4 motors,
        // and further derate (initial test by 5%) to account for some battery droop under heavy loads.
        // Max speed measured values 2/12/2023:  All 4 motors are between 4.6 an 4.7 meters/sec.  So use 4.5 as a conservative value
        public static final double kMaxSpeedMetersPerSecond = 4.5;          // CALIBRATED-3
        public static final double kNominalSpeedMetersPerSecond = 0.5*kMaxSpeedMetersPerSecond;
        // Max acceleration measured values 1/13/2023: FL = 28.073, FR = 26.343, BL = 18.482, BR = 19.289
        // Max acceleration measured 1/25/2023 (with ~80lbs on robot):  Average of 4 wheels = 10.0 m/sec^2
        // Max acceleration measured 2/12/2023 (with new drive gears):  Average ~11 m/sec^2.  Keep value at 10.0 for now.
        public static final double kMaxAccelerationMetersPerSecondSquare = 10; // CALIBRATED-3
        public static final double kNominalAccelerationMetersPerSecondSquare = 0.7*kMaxAccelerationMetersPerSecondSquare;
        public static final double kMaxTurningRadiansPerSecond = 11.0;   // CALIBRATED-3 took 633 degreesPerSecond and converted to radians and rounded down
        public static final double kNominalTurningRadiansPerSecond = Math.PI;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = 35.0;            // CALIBRATED-3 37.4 rad/sec^2
        public static final double kNominalAngularAccelerationRadiansPerSecondSquared = Math.PI;
        public static final double kVDrive = 0.2034; // CALIBRATED-3 = 0.2511.  in % output per meters per second.  Calibration says 0.2511, but better match on a trapezoid is 
        public static final double kADrive = 0.0;                   // TODO -- Calibrate
        public static final double kSDrive = 0.016; // CALIBRATED-3 = 0.016.  in % output

    }

      public static final class DriveConstants {
        // The locations of the wheels relative to the physical center of the robot, in meters.
        // X: + = forward.  Y: + = to the left
        // The order in which you pass in the wheel locations is the same order that
        // you will receive the module states when performing inverse kinematics. It is also expected that
        // you pass in the module states in the same order when calling the forward kinematics methods.
        // 0 = FrontLeft, 1 = FrontRight, 2 = BackLeft, 3 = BackRight
        public static final SwerveDriveKinematics kDriveKinematics =
            new SwerveDriveKinematics(
                new Translation2d(RobotDimensions.DRIVETRAIN_WHEELBASE_METERS / 2, RobotDimensions.DRIVETRAIN_TRACKWIDTH_METERS / 2),
                new Translation2d(RobotDimensions.DRIVETRAIN_WHEELBASE_METERS / 2, -RobotDimensions.DRIVETRAIN_TRACKWIDTH_METERS / 2),
                new Translation2d(-RobotDimensions.DRIVETRAIN_WHEELBASE_METERS / 2, RobotDimensions.DRIVETRAIN_TRACKWIDTH_METERS / 2),
                new Translation2d(-RobotDimensions.DRIVETRAIN_WHEELBASE_METERS / 2, -RobotDimensions.DRIVETRAIN_TRACKWIDTH_METERS / 2));

        // Update the offset angles in RobotPreferences (in Shuffleboard), not in this code!
        // After updating in RobotPreferences, you will need to re-start the robot code for the changes to take effect.
        // When calibrating offset, set the wheels to zero degrees with the bevel gear facing to the right
        public static double offsetAngleFrontLeftMotor = 0; // 92.3
        public static double offsetAngleFrontRightMotor = 0; // -12.8
        public static double offsetAngleBackLeftMotor = 0; // -107.6
        public static double offsetAngleBackRightMotor = 0; // -170.2
      }

      public static final class TrajectoryConstants {
        // Max error for robot rotation
        public static final double maxThetaErrorDegrees = 1.0;
        public static final double maxPositionErrorMeters = 0.02;

        // Feedback terms for holonomic drive controllers
        public static final double kPXController = 1;       // X-velocity controller:  kp.  Units = (meters/sec of velocity) / (meters of position error)
        public static final double kPYController = 1;       // Y-velocity controller:  kp.  Units = (meters/sec of velocity) / (meters of position error)
        public static final double kPThetaController = 3;   // Theta-velocity controller:  kp.  Units = (rad/sec of velocity) / (radians of angle error)

        public static final TrajectoryConfig swerveTrajectoryConfig =
            new TrajectoryConfig(
                    SwerveConstants.kNominalSpeedMetersPerSecond,
                    SwerveConstants.kNominalAccelerationMetersPerSecondSquare)
                .setKinematics(DriveConstants.kDriveKinematics);

        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            SwerveConstants.kNominalTurningRadiansPerSecond, SwerveConstants.kNominalAngularAccelerationRadiansPerSecondSquared);

        /* Constraint for the DriveToPose motion profile for distance being travelled */
        public static final TrapezoidProfileBCR.Constraints kDriveProfileConstraints =
        new TrapezoidProfileBCR.Constraints(
            SwerveConstants.kNominalSpeedMetersPerSecond, SwerveConstants.kNominalAccelerationMetersPerSecondSquare);
      }

      public static final class WristConstants {
        public static double offsetAngleWrist = 0;
        // Wrist Angles (in degrees)
        // TODO Find these values
        public static final double max = 113.0;		// Location of upper limit switch for auto calibration
        public static final double stowed = 0; // Starting angle
        // public static final double wristKeepOut = 28.0; // Max angle to avoid interference with elevator(Maybe necessary)
        public static final double scoreCargo = 100; // Angle to score cargo
        public static final double loadCargoStation = -45; // Angle to pick up cargo from loading station
        public static final double loadCargoGround = 0; // Angle to pick up cargo from ground intake
        // public static final double wristMinWhenElevatorLow = -45.0;   // If the elevator is in the groundCargo position, don't move the wrist below this!
        public static final double straight = 90;	//  needed to bias upward to account for sag and insure that hatch cover gripper engages first
        // public static final double wristDown = -60.0;		// In this position, elevator must be able to go to groundCargo (Not sure we need this)
        public static final double vision = 60;    // wrist angle for optimal vision tracking (Maybe to keep out of the way of camera? Might not be necessary)
        public static final double min = -61.0;			// Location of lower limit switch for auto calibration
        // public static final double wristMax = 113.0;		// Location of upper limit switch for auto calibration
        // public static final double wristStowed = 110.0;
        // public static final double wristKeepOut = 28.0; // Max angle to avoid interference with elevator or climber
        // public static final double wristUp = 15.0;
        // public static final double wristStraight = -1.0;	//  needed to bias upward to account for sag and insure that hatch cover gripper engages first
        // public static final double wristVision = -5.0;    // wrist angle for optimal vision tracking
        // public static final double wristCargoShot = -30.0;	// Angle for wrist for cargo ship ball shot
        // public static final double wristLowerCrashWhenElevatorLow = -45.0;   // If the elevator is in the low position, don't move the wrist below this!
        // public static final double wristDown = -60.0;		 // In this position, elevator must be able to go to groundCargo
        // public static final double wristMin = -61.0;			// Location of lower limit switch for auto calibration
        public enum WristAngle {stowed, loadCargoStation, loadCargoGround, scoreCargo, straight, vision}
        // public enum WristAngle {stowed, up, straight, scoreCargo, vision, down}
        public static final double encoderTicksPerRevolution = 4096.0; // Neo ticks per revolution?
            
        // Wrist regions
        public enum WristRegion {back, main, uncalibrated}  
      }

      public static final class ElevatorConstants {
        public static final double kEncoderCPR = 2048.0;                // CALIBRATED = 2048.  Encoder counts per revolution of FalconFX motor pinion gear
        public static final double kElevGearRatio = (12.0 / 1.0);        // CALIBRATED.  Gear reduction ratio between Falcon and gear driving the elevator
        public static final double kElevGearDiameterInches = 1.300;       // CALIBRATED.  Diameter of the gear driving the elevator in inches.  Per CAD = 1.273.  Calibrated = 1.300.
        public static final double kElevEncoderInchesPerTick = (kElevGearDiameterInches * Math.PI) / kEncoderCPR / kElevGearRatio;

        public static final double maxUncalibratedPercentOutput = 0.10;     // TODO CALIBRATE

        // Elevator regions
        public enum ElevatorRegion {
            bottom,     // In the elevator bottom region, the wrist may be in any wrist region.
            main,       // In the elevator main region, the wrist must be in the wrist main region (not allowed to go to wrist back region).
            uncalibrated;       // Unknown region, elevator is not calibrated.
        }     
        // Elevator region boundaries
        public static final double mainBottom = 2.0;      // Boundary between bottom and main regions.  TODO CALIBRATE

        // Elevator pre-defined positions (in inches from bottom of elevator)
        public enum ElevatorPosition {
            lowerLimit(0.0),
            bottom(0.0),            // TODO CALIBRATE
            loadingStation(20.0),   // TODO CALIBRATE
            scoreMid(25.0),         // TODO CALIBRATE
            scoreHigh(30.0),        // TODO CALIBRATE
            upperLimit(35.0);       // TODO CALIBRATE
        
            @SuppressWarnings({"MemberName", "PMD.SingularField"})
            public final double value;
            ElevatorPosition(double value) { this.value = value; }
        }
      }
}
