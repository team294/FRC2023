package frc.robot.utilities;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.CoordType;
import frc.robot.Constants.StopType;
import frc.robot.commands.*;
import frc.robot.commands.autos.*;
import frc.robot.subsystems.*;
import frc.robot.utilities.TrajectoryCache.TrajectoryType;


/**
 * Selects the auto routine based upon input from the shuffleboard
 */
public class AutoSelection {

	public static final int EXAMPLE = 0;
	public static final int STRAIGHT = 1;
	public static final int LEAVE_COMMUNITY = 2;
	public static final int RIGHT_ONE_CONE_BALANCE = 3;
	public static final int LEFT_ONE_CONE_BALANCE = 4;
	public static final int MIDDLE_ONE_CONE_BALANCE = 5;

	private AllianceSelection allianceSelection;
	private TrajectoryCache trajectoryCache;
	private SendableChooser<Integer> autoChooser = new SendableChooser<>();
	
	/**
	 * AutoSelection constructor for command group
	 * Sets up autoPlan widget 
	 */  	
	public AutoSelection(TrajectoryCache trajectoryCache, AllianceSelection allianceSelection, FileLog log) {
		this.trajectoryCache = trajectoryCache;
		this.allianceSelection = allianceSelection;

		// auto selections
		autoChooser.setDefaultOption("Example Auto", EXAMPLE);
		autoChooser.addOption("Straight", STRAIGHT);
		autoChooser.addOption("Leave Community", LEAVE_COMMUNITY);
		autoChooser.addOption("Right One Cone Balance", RIGHT_ONE_CONE_BALANCE);
		autoChooser.addOption("Left One Cone Balance", LEFT_ONE_CONE_BALANCE);
		autoChooser.addOption("Middle One Cone Balance", MIDDLE_ONE_CONE_BALANCE);
	
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
	public Command getAutoCommand(DriveTrain driveTrain, FileLog log) {
		Command autonomousCommand = null;

		// Get parameters from Shuffleboard
		int autoPlan = autoChooser.getSelected();

		double waitTime = SmartDashboard.getNumber("Autonomous delay", 0);
		waitTime = MathUtil.clamp(waitTime, 0, 15);		// make sure autoDelay isn't negative and is only active during auto

		if (autoPlan == EXAMPLE) {
		 	log.writeLogEcho(true, "AutoSelect", "run Example Auto");
			autonomousCommand = new SequentialCommandGroup(new WaitCommand(waitTime), new ExampleAuto(driveTrain));
		}

		if (autoPlan == STRAIGHT) {
			log.writeLogEcho(true, "AutoSelect", "run Straight");
		   autonomousCommand = new SequentialCommandGroup(new WaitCommand(waitTime), 
		   		new DriveTrajectory(CoordType.kRelative, StopType.kBrake, 
						trajectoryCache.cache[TrajectoryType.test.value], driveTrain, log)
		   );
	   }

	   if (autoPlan == LEAVE_COMMUNITY) {
			log.writeLogEcho(true, "AutoSelect", "run Leave Community");
	   		autonomousCommand = new SequentialCommandGroup(new WaitCommand(waitTime),
				new DriveTrajectory(CoordType.kAbsoluteResetPose, StopType.kBrake, 
						trajectoryCache.cache[TrajectoryType.LeaveCommunity.value], driveTrain, log)
	   		);
   	   }

	   if(autoPlan == RIGHT_ONE_CONE_BALANCE){
			log.writeLogEcho(true, "AutoSelect", "run Right One Cone Balance");
			autonomousCommand = new SequentialCommandGroup(new WaitCommand(waitTime),
				new DriveTrajectory(CoordType.kAbsoluteResetPoseTol, StopType.kBrake, 
					((allianceSelection.getAlliance() == Alliance.Red) ? 
						trajectoryCache.cache[TrajectoryType.RightOuterOneConeBalanceRed.value]:
						trajectoryCache.cache[TrajectoryType.RightOuterOneConeBalanceBlue.value] ), 
					driveTrain, log)
			);
	   }

	   if(autoPlan == LEFT_ONE_CONE_BALANCE){
			log.writeLogEcho(true, "AutoSelect", "run Left One Cone Balance");
			autonomousCommand = new SequentialCommandGroup(new WaitCommand(waitTime),
				new DriveTrajectory(CoordType.kAbsoluteResetPoseTol, StopType.kBrake, 
					((allianceSelection.getAlliance() == Alliance.Red) ? 
						trajectoryCache.cache[TrajectoryType.LeftOuterOneConeBalanceRed.value]:
						trajectoryCache.cache[TrajectoryType.LeftOuterOneConeBalanceBlue.value] ), 
					driveTrain, log)
			);
	   }

	   if(autoPlan == MIDDLE_ONE_CONE_BALANCE){
			log.writeLogEcho(true, "AutoSelect", "run Middle One Cone Balance");
			autonomousCommand = new SequentialCommandGroup(new WaitCommand(waitTime),
				new DriveTrajectory(CoordType.kAbsoluteResetPoseTol, StopType.kBrake, 
					((allianceSelection.getAlliance() == Alliance.Red) ? 
						trajectoryCache.cache[TrajectoryType.MiddleOuterOneConeBalanceRed.value]:
						trajectoryCache.cache[TrajectoryType.MiddleOuterOneConeBalanceBlue.value] ), 
					driveTrain, log)
			);
	   }

   if (autonomousCommand == null) {
			log.writeLogEcho(true, "AutoSelect", "No autocommand found");
			autonomousCommand = new WaitCommand(1);
		}

		return autonomousCommand;
	}

}
