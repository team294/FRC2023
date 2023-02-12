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
        kRelative(0),
        kAbsolute(1),
        kAbsoluteResetPose(2);
    
        @SuppressWarnings({"MemberName", "PMD.SingularField"})
        public final int value;
        CoordType(int value) { this.value = value; }
    }

    /**
     * Options to select driving stopping types.
     */
    public enum StopType {
        kNoStop(0),
        kCoast(1),
        kBrake(2);
    
        @SuppressWarnings({"MemberName", "PMD.SingularField"})
        public final int value;
        StopType(int value) { this.value = value; }
    }


    public static final class Ports{

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

        public static final int CANGrabber = 44;

        public static final int CANElevatorMotor1 = 13;
        public static final int CANElevatorMotor2 = 14;
        public static final int CANWristMotor = 15;
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
        public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.59127;      // CALIBRATED-2 = 0.59127 (based on robot rotating in place).  CAD geometry = 0.57785.
        //front-back distance between the drivetrain wheels; should be measured from center to center
        public static final double DRIVETRAIN_WHEELBASE_METERS = 0.59127;       // CALIBRATED-2 = 0.59127 (based on robot rotating in place).  CAD geometry = 0.57785.

    }

    public static final class SwerveConstants {
        // Encoder calibration to meters travelled or wheel facing degrees
        public static final double kEncoderCPR = 2048.0;                // CALIBRATED-2 = 2048.  Encoder counts per revolution of FalconFX motor pinion gear
        public static final double kDriveGearRatio = (8.14 / 1.0);      // CALIBRATED-2 = 8.14/1.0.  Team364 (MK3i?) = 6.86:1.  Mk4i = 8.14 : 1
        public static final double kTurningGearRatio = (150.0/7.0 / 1.0); // CALIBRATED-2 = 150.0/7.0.  Team364 (MK3i?) = 12.8:1.  Mk4i = 150/7 : 1
        public static final double kWheelDiameterMeters = 0.09953;        // CALIBRATED-2 = 0.102.  Depends a little on the tread wear!
        public static final double kDriveEncoderMetersPerTick = (kWheelDiameterMeters * Math.PI) / kEncoderCPR / kDriveGearRatio;
        public static final double kTurningEncoderDegreesPerTick = 360.0/kEncoderCPR / kTurningGearRatio;
        
        // Robot calibration for feed-forward and max speeds
        // Max speed is used to keep each motor from maxing out, which preserves ratio between motors 
        // and ensures that the robot travels in the requested direction.  So, use min value of all 4 motors,
        // and further derate (initial test by 5%) to account for some battery droop under heavy loads.
        // Max speed measured values 1/25/2023:  FL = 3.98, FR  = 3.97, BL = 3.98, BR = 3.95
        public static final double kMaxSpeedMetersPerSecond = 3.8;          // CALIBRATED-2
        public static final double kNominalSpeedMetersPerSecond = 0.5*kMaxSpeedMetersPerSecond;
        // Max acceleration measured values 1/13/2023: FL = 28.073, FR = 26.343, BL = 18.482, BR = 19.289
        // Max acceleration measured 1/25/2023 (with ~80lbs on robot):  Average of 4 wheels = 10.0 m/sec^2
        public static final double kMaxAccelerationMetersPerSecondSquare = 10; // CALIBRATED-2
        public static final double kNominalAccelerationMetersPerSecondSquare = 0.7*kMaxAccelerationMetersPerSecondSquare;
        // Max turn velocity degrees per second measured values 1/13/2023: FL = 1744.629, FR = 1762.207, BL = 1736.719, BR = 2085.645
        public static final double kMaxTurningRadiansPerSecond = 9.1;   // CALIBRATED-2 took 528 degreesPerSecond and converted to radians
        public static final double kNominalTurningRadiansPerSecond = Math.PI;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = 30.0;            // CALIBRATED-2 31.7 rad/sec^2
        public static final double kNominalAngularAccelerationRadiansPerSecondSquared = Math.PI;
        public static final double kVDrive = 0.248; // CALIBRATED-2 = 0.248.  in % output per meters per second
        public static final double kADrive = 0.0;                   // TODO -- Calibrate
        public static final double kSDrive = 0.017; // CALIBRATED-2 = 0.017.  in % output

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
        public static double offsetAngleFrontLeftMotor = 0; // 92.2
        public static double offsetAngleFrontRightMotor = 0; // -12.5
        public static double offsetAngleBackLeftMotor = 0; // -106.6
        public static double offsetAngleBackRightMotor = 0; // 157.5
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
            // Wrist Angles (in degrees)
            public static final double wristMax = 113.0;		// Location of upper limit switch for auto calibration
            public static final double wristStowed = 110.0;
            public static final double wristKeepOut = 28.0; // Max angle to avoid interference with elevator or climber
            public static final double wristUp = 15.0;
            public static final double wristStraight = -1.0;	//  needed to bias upward to account for sag and insure that hatch cover gripper engages first
            public static final double wristVision = -5.0;    // wrist angle for optimal vision tracking
            public static final double wristCargoShot = -30.0;	// Angle for wrist for cargo ship ball shot
            public static final double wristLowerCrashWhenElevatorLow = -45.0;   // If the elevator is in the low position, don't move the wrist below this!
            public static final double wristDown = -60.0;		// TODO Should be -59.0? // In this position, elevator must be able to go to groundCargo
            public static final double wristMin = -61.0;			// Location of lower limit switch for auto calibration
            public enum WristAngle {stowed, up, straight, cargoShot, vision, down}
            public static final double encoderTicksPerRevolution = 4096.0;
            
        }

      public static final class ElevatorConstants {
        public static final double hatchLow = 19.0;
        public static final double hatchMid = 48.5;
        public static final double hatchHigh = 69.7;		// was 72.8
        public static final double cargoShipCargo = 43.0;   // Was 34.75
        public static final double rocketBallOffset = 2;  // Ball intake is higher than the disc grabber (low position only)
        public static final double loadCargo = 44.125;
        public static final double groundCargo = 16.5;  		// At this level, wrist must be able to go to wristDown  // TODO should this be the same as elevatorWristStow (at the hard stop)?
        public static final double encoderTicksPerRevolution = 4096.0;

        public enum ElevatorPosition {bottom, vision, wristStow, hatchLow, hatchMid, hatchHigh, cargoShipCargo, loadCargo, groundCargo}        
      }
}
