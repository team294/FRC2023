package frc.robot.commands.autos;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utilities.FileLog;
import frc.robot.utilities.StringUtil;

import static frc.robot.Constants.DriveConstants.kPDriveBalance;;

public class SmartBalance extends CommandBase {
  private final FileLog log;
  private final DriveTrain drivetrain;
  private final double speed;
  private double initPitch = 0;
  private double pitch;
  private double maxPitch;
  private double initialX;
  private boolean ascended;

  /** Creates a new SmartBalance. This command will drive robot up a ramp a given distance
   *  or until the ramp tilts. It will print the maximum slope of the ramp.
   * 
   * @param speed The speed at which the robot will drive
   * @param drive The DriveTrain subsystem on which this command will run
  */

  /**
   * Creates a new SmartBalance. This command will drive robot up a ramp a given distance
   * or until the ramp tilts. It will print the maximum slope of the ramp.
   * @param speed the speed (m/s) at which the robot will drive, field relative
   * @param drivetrain the DriveTrain subsystem on which this command will run
   * @param log FileLog used for logging
   */
  public SmartBalance(double speed, DriveTrain drivetrain, FileLog log) {
    this.speed = speed;
    this.drivetrain = drivetrain;
    this.log = log;

    addRequirements(drivetrain);
  }

    // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initPitch = drivetrain.getGyroPitch();
    ActiveBalance.goalAngle = initPitch;

    initialX = drivetrain.getPose().getX();
    maxPitch = Math.abs(initPitch);

    ascended = Math.abs(initPitch) > 10;
    SmartDashboard.putNumber("Initial Pitch", initPitch);
  }

  // drives field relative at speed towards the charging station
  @Override
  public void execute() {  
    drivetrain.drive(speed, 0.0, 0.0, true, false);

    if (!ascended) ascended = Math.abs(pitch) > 23;
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stopMotors();
  }

  // Returns true when the robot has 
  @Override
  public boolean isFinished() {
    pitch = drivetrain.getGyroPitch();

    SmartDashboard.putNumber("Pitch", pitch);
    log.writeLog(false, "SmartBalance", "isFinished", StringUtil.buildString("Pitch", pitch, "Max Pitch", maxPitch, "Odometry X", drivetrain.getPose().getX(), "Speed", speed));

    if (Math.abs(pitch) > maxPitch) maxPitch = Math.abs(pitch);

    return ascended && Math.abs(pitch) < 16 && drivetrain.getPose().getX() - initialX > 2.1;    
  }
}