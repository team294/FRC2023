package frc.robot.utilities;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.CoordType;
import frc.robot.Constants.StopType;
import frc.robot.commands.DriveTrajectory;
import frc.robot.commands.ExampleAuto;
import frc.robot.subsystems.*;
import frc.robot.utilities.TrajectoryCache.TrajectoryType;


/**
 * Selects the auto routine based upon input from the shuffleboard
 */
public class AutoSelection {

	public static final int EXAMPLE = 0;
	public static final int STRAIGHT = 1;
	
	private TrajectoryCache trajectoryCache;
	private SendableChooser<Integer> autoChooser = new SendableChooser<>();
	
	/**
	 * AutoSelection constructor for command group
	 * Sets up autoPlan widget 
	 */  	
	public AutoSelection(TrajectoryCache trajectoryCache, FileLog log) {
		this.trajectoryCache = trajectoryCache;

		// auto selections
		autoChooser.setDefaultOption("Example Auto", EXAMPLE);
		autoChooser.addOption("Straight", STRAIGHT);
	
		// show auto selection widget on Shuffleboard
		SmartDashboard.putData("Autonomous routine", autoChooser);

		// show auto parameters on Shuffleboard
		SmartDashboard.putNumber("Autonomous delay", 0);
		SmartDashboard.putBoolean("Autonomous use vision", false);
	}

	/**
	 * Gets the auto command based upon input from the shuffleboard
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
		   			new DriveTrajectory(CoordType.kRelative, StopType.kBrake, trajectoryCache.cache[TrajectoryType.test.value], driveTrain, log)
		   );
	   }

	   if (autonomousCommand == null) {
			log.writeLogEcho(true, "AutoSelect", "No autocommand found");
			autonomousCommand = new WaitCommand(1);
		}

		return autonomousCommand;
	}

}
