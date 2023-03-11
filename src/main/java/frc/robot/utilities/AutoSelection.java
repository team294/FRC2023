package frc.robot.utilities;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.commands.*;
import frc.robot.commands.autos.*;
import frc.robot.subsystems.*;


/**
 * Selects the auto routine based upon input from the shuffleboard
 */
public class AutoSelection {

	public static final int NONE = 0;
	public static final int STRAIGHT = 1;
	public static final int LEAVE_COMMUNITY = 2;
	public static final int RIGHT_ONE_CONE_BALANCE = 3;
	public static final int LEFT_ONE_CONE_BALANCE = 4;
	public static final int MIDDLE_ONE_CONE_BALANCE = 5;
	public static final int MIDDLE_BALANCE = 6;
	public static final int CONE_LEAVE_NEAR_WALL = 7;
	public static final int SCORE_CONE = 8;
	public static final int CONE_BALANCE_4TOWALL = 9;
	public static final int BALANCE_4TOWALL = 10;
	public static final int CONE_LEAVE_NEAR_WALL_CROSS = 11;

	private final AllianceSelection allianceSelection;
	private final TrajectoryCache trajectoryCache;
	private final Field field;
	private final double interimPositionErrorMeters = TrajectoryConstants.maxPositionErrorMeters + Units.inchesToMeters(4);
	private final double interimThetaErrorDegrees = TrajectoryConstants.maxThetaErrorDegrees + 1.0;
	private SendableChooser<Integer> autoChooser = new SendableChooser<>();
	
	/**
	 * AutoSelection constructor for command group
	 * Sets up autoPlan widget 
	 */  	
	public AutoSelection(TrajectoryCache trajectoryCache, AllianceSelection allianceSelection, Field field, FileLog log) {
		this.trajectoryCache = trajectoryCache;
		this.allianceSelection = allianceSelection;
		this.field = field;

		// auto selections
		autoChooser.setDefaultOption("None", NONE);
		autoChooser.addOption("Score Cone", SCORE_CONE);
		autoChooser.addOption("Cone Leave NearWall", CONE_LEAVE_NEAR_WALL);
		autoChooser.addOption("Cone Balance 4ToWall", CONE_BALANCE_4TOWALL);
		autoChooser.addOption("Balance 4ToWall", BALANCE_4TOWALL);
		autoChooser.addOption("Cone Nearwall Cross", CONE_LEAVE_NEAR_WALL_CROSS);

		// autoChooser.addOption("Straight", STRAIGHT);
		// autoChooser.addOption("Leave Community", LEAVE_COMMUNITY);
		// autoChooser.addOption("Right One Cone Balance", RIGHT_ONE_CONE_BALANCE);
		// autoChooser.addOption("Left One Cone Balance", LEFT_ONE_CONE_BALANCE);
		// autoChooser.addOption("Middle One Cone Balance", MIDDLE_ONE_CONE_BALANCE);
		// autoChooser.addOption("Middle Balance", MIDDLE_BALANCE);
	
		// show auto selection widget on Shuffleboard
		SmartDashboard.putData("Autonomous routine", autoChooser);

		// show auto parameters on Shuffleboard
		SmartDashboard.putNumber("Autonomous delay", 0);
		SmartDashboard.putBoolean("Autonomous use vision", false);
	}

	/**
	 * Gets the auto command based upon input from the shuffleboard.
	 * This method is designed to be called at AutonomousInit by Robot.java.
	 * 
	 * @param driveTrain The driveTrain that will be passed to the auto command
	 * @param log        The filelog to write the logs to
	 * @return the command to run
	 */
	public Command getAutoCommand(Elevator elevator, Wrist wrist, Manipulator manipulator, DriveTrain driveTrain, LED led, FileLog log) {
		Command autonomousCommand = null;

		// Get parameters from Shuffleboard
		int autoPlan = autoChooser.getSelected();

		double waitTime = SmartDashboard.getNumber("Autonomous delay", 0);
		waitTime = MathUtil.clamp(waitTime, 0, 15);		// make sure autoDelay isn't negative and is only active during auto

		if (autoPlan == NONE) {
			// Starting position = facing drivers
			log.writeLogEcho(true, "AutoSelect", "run None");
			autonomousCommand = new DriveResetPose(180, false, driveTrain, log);
		}

		if (autoPlan == SCORE_CONE) {
			// Starting position = facing drivers, against a cone scoring location
			log.writeLogEcho(true, "AutoSelect", "run Score Cone");
	   		autonomousCommand = new SequentialCommandGroup(new WaitCommand(waitTime),
			   	new DriveResetPose(180, false, driveTrain, log),
				new AutoScoreConeHigh(elevator, wrist, manipulator, led, log)
	   		);
   	   	}

		if (autoPlan == CONE_LEAVE_NEAR_WALL) {
			// Starting position = facing drivers, against scoring position closest to wall
			log.writeLogEcho(true, "AutoSelect", "run Cone Leave Near Wall");
			Pose2d posScoreInitial, posLeaveFinal;
			if (allianceSelection.getAlliance() == Alliance.Red) {
				posScoreInitial = field.getFinalColumn(9);
			} else {
				posScoreInitial = field.getFinalColumn(1);
			}
			// Travel  4.4 m in +X from starting position
			posLeaveFinal = MathBCR.translate(posScoreInitial, 4.4, 0);

	   		autonomousCommand = new SequentialCommandGroup(new WaitCommand(waitTime),
			    new DriveResetPose(posScoreInitial, true, driveTrain, log),
			    new AutoScoreConeHigh(elevator, wrist, manipulator, led, log),
				new DriveToPose(posLeaveFinal, driveTrain, log)
	   		);
   	   	}

		if (autoPlan == CONE_LEAVE_NEAR_WALL_CROSS) {
			// Starting position = facing drivers, against scoring position closest to wall
			log.writeLogEcho(true, "AutoSelect", "run Cone Leave Near Wall");
			Pose2d posScoreInitial, posLeave, posCross, posFinal;
			if (allianceSelection.getAlliance() == Alliance.Red) {
				posScoreInitial = field.getFinalColumn(9);			// 1.77165, 7.490968, 180
				// Travel  4.4 m in +X from starting position
				posLeave = MathBCR.translate(posScoreInitial, 4.4, 0);  // 6.17165, 7.490968, 180
				// Travel in Y to cross the field to the pickup side
				posCross = new Pose2d(6.3, 2.2, Rotation2d.fromDegrees(180.0));
				// Spin 180
				posFinal = new Pose2d(7.0, 2.2, Rotation2d.fromDegrees(0.0));
			} else {
				posScoreInitial = field.getFinalColumn(1);			// 1.77165, 0.512826, 180
				// Travel  4.4 m in +X from starting position
				posLeave = MathBCR.translate(posScoreInitial, 4.4, 0);		// 6.17165, 0.512826, 180
				// Travel in Y to cross the field to the pickup side
				posCross = new Pose2d(6.3, 6.0, Rotation2d.fromDegrees(180.0));
				// Spin 180
				posFinal = new Pose2d(7.0, 6.0, Rotation2d.fromDegrees(0.0));
			}

	   		autonomousCommand = new SequentialCommandGroup(new WaitCommand(waitTime),
			    new DriveResetPose(posScoreInitial, true, driveTrain, log),
			    new AutoScoreConeHigh(elevator, wrist, manipulator, led, log),
				new DriveToPose(posLeave, SwerveConstants.kNominalSpeedMetersPerSecond, SwerveConstants.kNominalAccelerationMetersPerSecondSquare,
					interimPositionErrorMeters, interimThetaErrorDegrees, driveTrain, log),
				new DriveToPose(posCross, SwerveConstants.kNominalSpeedMetersPerSecond, SwerveConstants.kNominalAccelerationMetersPerSecondSquare,
					interimPositionErrorMeters, interimThetaErrorDegrees, driveTrain, log),
				new DriveToPose(posFinal, SwerveConstants.kNominalSpeedMetersPerSecond, SwerveConstants.kNominalAccelerationMetersPerSecondSquare,
					TrajectoryConstants.maxPositionErrorMeters, TrajectoryConstants.maxThetaErrorDegrees, driveTrain, log)
	   		);
   	   	}

		if (autoPlan == CONE_BALANCE_4TOWALL) {
			// Starting position = facing drivers, 4th scoring position from wall
			log.writeLogEcho(true, "AutoSelect", "run Cone Balance 4ToWall");
			// Pose2d posCommunityInitial = field.getStationInitial(2);
			// Pose2d posCommunityFinal = field.getStationCenter(2);
			// Pose2d posCommunityFinal = translate(field.getStationCenter(2), 1.5, 0.0);		// overdrive due to wheel slip when climbing on charging station
			Pose2d posScoreInitial;
			if (allianceSelection.getAlliance() == Alliance.Red) {
				posScoreInitial = field.getFinalColumn(6);
			} else {
				posScoreInitial = field.getFinalColumn(4);
			}

	   		autonomousCommand = new SequentialCommandGroup(new WaitCommand(waitTime),
			    new DriveResetPose(posScoreInitial, true, driveTrain, log),
				new AutoScoreConeHigh(elevator, wrist, manipulator, led, log),
				new DriveUpChargingStation(0.5, driveTrain, log),
				new ActiveBalance(driveTrain, log)
				//new AutoBalance(posCommunityInitial, posCommunityFinal, driveTrain, log)
	   		);
   	   	}

		if (autoPlan == BALANCE_4TOWALL) {
			// Starting position = facing drivers, 4th scoring position from wall
			log.writeLogEcho(true, "AutoSelect", "run Balance 4ToWall");
			
			Pose2d posScoreInitial;
			if (allianceSelection.getAlliance() == Alliance.Red) {
				posScoreInitial = field.getFinalColumn(6);
			} else {
				posScoreInitial = field.getFinalColumn(4);
			}

	   		autonomousCommand = new SequentialCommandGroup(new WaitCommand(waitTime),
			    new DriveResetPose(posScoreInitial, true, driveTrain, log),
				new DriveUpChargingStation(0.5, driveTrain, log),
				new ActiveBalance(driveTrain, log)
	   		);
   	   	}

		// 	if (autoPlan == STRAIGHT) {
	// 		log.writeLogEcho(true, "AutoSelect", "run Straight");
	// 	   autonomousCommand = new SequentialCommandGroup(new WaitCommand(waitTime), 
	// 	   		new DriveTrajectory(CoordType.kRelative, StopType.kBrake, 
	// 					trajectoryCache.cache[TrajectoryType.test.value], driveTrain, log)
	// 	   );
	//    }

	//    if (autoPlan == LEAVE_COMMUNITY) {
	// 		log.writeLogEcho(true, "AutoSelect", "run Leave Community");
	//    		autonomousCommand = new SequentialCommandGroup(new WaitCommand(waitTime),
	// 			new DriveTrajectory(CoordType.kAbsoluteResetPose, StopType.kBrake, 
	// 					trajectoryCache.cache[TrajectoryType.LeaveCommunity.value], driveTrain, log)
	//    		);
   	//    }

	//    if(autoPlan == RIGHT_ONE_CONE_BALANCE){
	// 		log.writeLogEcho(true, "AutoSelect", "run Right One Cone Balance");
	// 		autonomousCommand = new SequentialCommandGroup(new WaitCommand(waitTime),
	// 			new DriveTrajectory(CoordType.kAbsoluteResetPoseTol, StopType.kBrake, 
	// 				((allianceSelection.getAlliance() == Alliance.Red) ? 
	// 					trajectoryCache.cache[TrajectoryType.RightOuterOneConeBalanceRed.value]:
	// 					trajectoryCache.cache[TrajectoryType.RightOuterOneConeBalanceBlue.value] ), 
	// 				driveTrain, log)
	// 		);
	//    }

	//    if(autoPlan == LEFT_ONE_CONE_BALANCE){
	// 		log.writeLogEcho(true, "AutoSelect", "run Left One Cone Balance");
	// 		autonomousCommand = new SequentialCommandGroup(new WaitCommand(waitTime),
	// 			new DriveTrajectory(CoordType.kAbsoluteResetPoseTol, StopType.kBrake, 
	// 				((allianceSelection.getAlliance() == Alliance.Red) ? 
	// 					trajectoryCache.cache[TrajectoryType.LeftOuterOneConeBalanceRed.value]:
	// 					trajectoryCache.cache[TrajectoryType.LeftOuterOneConeBalanceBlue.value] ), 
	// 				driveTrain, log)
	// 		);
	//    }

	//    if(autoPlan == MIDDLE_ONE_CONE_BALANCE){
	// 		log.writeLogEcho(true, "AutoSelect", "run Middle One Cone Balance");
	// 		autonomousCommand = new SequentialCommandGroup(new WaitCommand(waitTime),
	// 			new DriveTrajectory(CoordType.kAbsoluteResetPoseTol, StopType.kBrake, 
	// 				((allianceSelection.getAlliance() == Alliance.Red) ? 
	// 					trajectoryCache.cache[TrajectoryType.MiddleOuterOneConeBalanceRed.value]:
	// 					trajectoryCache.cache[TrajectoryType.MiddleOuterOneConeBalanceBlue.value] ), 
	// 				driveTrain, log)
	// 		);
	//    }

	//    if(autoPlan == MIDDLE_BALANCE){
	// 	log.writeLogEcho(true, "AutoSelect", "run Middle Balance");
	// 	autonomousCommand = new SequentialCommandGroup(new WaitCommand(waitTime),
	// 		new DriveTrajectory(CoordType.kAbsoluteResetPoseTol, StopType.kBrake, 
	// 			((allianceSelection.getAlliance() == Alliance.Red) ? 
	// 				trajectoryCache.cache[TrajectoryType.CenterBalanceRed.value]:
	// 				trajectoryCache.cache[TrajectoryType.CenterBalanceBlue.value] ), 
	// 			driveTrain, log)
	// 	);
	//    }
	   

   if (autonomousCommand == null) {
			log.writeLogEcho(true, "AutoSelect", "No autocommand found");
			autonomousCommand = new WaitCommand(1);
		}

		return autonomousCommand;
	}

}
