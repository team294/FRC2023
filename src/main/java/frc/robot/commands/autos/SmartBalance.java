package frc.robot.commands.autos;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import static frc.robot.Constants.DriveConstants.kPDriveBalance;;

public class SmartBalance extends CommandBase {
  private final DriveTrain drivetrain;
  private final double speed;
  private double initPitch = 0;
  private double pitch;
  private double tolerance;
  private boolean ascend;

  private int toleranceCounter = 0;

  /** Creates a new SmartBalance. This command will drive robot up a ramp a given distance
   *  or until the ramp tilts. It will print the maximum slope of the ramp.
   * 
   * @param speed The speed at which the robot will drive
   * @param drive The DriveTrain subsystem on which this command will run
  */
  public SmartBalance(double speed, double tolerance, DriveTrain drivetrain) {
    this.speed = speed;
    this.tolerance = tolerance;
    this.drivetrain = drivetrain;

    addRequirements(drivetrain);
  }

    // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initPitch = drivetrain.getGyroPitch();
    ascend = Math.abs(initPitch) > 5;
    toleranceCounter = 0;
    SmartDashboard.putNumber("Initial Pitch", initPitch);
  }

  private double getError() {
    if (!ascend) {

    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  // It is using the encoders to keep itself straight
  @Override
  public void execute() {    
    drivetrain.drive(speed - kPDriveBalance * getError(), 0.0, 0.0, true, false);
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    pitch = drivetrain.getGyroPitch();
    System.out.println( pitch );
    SmartDashboard.putNumber("Pitch", pitch);
 
    if (ascend) {
      if(pitch < 1) done = true;  // reached the fulcrum
      else done = false;
    }
   
    if (!elevator.encoderCalibrated() ||         // End immediately if encoder can't read
      Math.abs(elevator.getElevatorPos() - target) <= 0.5) {
        toleranceCounter++;
    }
    return (toleranceCounter > 5);
    
  }
}