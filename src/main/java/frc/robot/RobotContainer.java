// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.Constants.CoordType;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.StopType;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
// import frc.robot.triggers.*;
import frc.robot.utilities.*;
import frc.robot.utilities.TrajectoryCache.TrajectoryType;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Define robot key utilities (DO THIS FIRST)
  private final FileLog log = new FileLog("A1");
  private final AllianceSelection allianceSelection = new AllianceSelection(log);
  private final Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);

  // Define robot subsystems  
  private final DriveTrain driveTrain = new DriveTrain(log);
  private final Grabber grabber = new Grabber("Grabber", log);
  private final Manipulator manipulator = new Manipulator(log);
  private final LED led = new LED();

  // Define other utilities
  private final TrajectoryCache trajectoryCache = new TrajectoryCache(log);
  private final AutoSelection autoSelection = new AutoSelection(trajectoryCache, log);
  private final Field field = new Field(driveTrain, manipulator, allianceSelection, log);

  // Define controllers
  // private final Joystick xboxController = new Joystick(OIConstants.usbXboxController); //assuming usbxboxcontroller is int
  private final Joystick leftJoystick = new Joystick(OIConstants.usbLeftJoystick);
  private final Joystick rightJoystick = new Joystick(OIConstants.usbRightJoystick);
  private final Joystick coPanel = new Joystick(OIConstants.usbCoPanel);

  private final CommandXboxController xboxController = new CommandXboxController(OIConstants.usbXboxController);
  private boolean rumbling = false;

  // Set to this pattern when the robot is disabled
  private final Command patternTeamMoving = new LEDSetPattern(LED.teamMovingColorsLibrary, 0, 0.5, led, log);
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureButtonBindings(); // configure button bindings
    configureShuffleboard(); // configure shuffleboard

    driveTrain.setDefaultCommand(new DriveWithJoystick(leftJoystick, rightJoystick, driveTrain, log));
  }

    /**
   * Define Shuffleboard mappings.
   */
  private void configureShuffleboard() {

    // display sticky faults
    RobotPreferences.showStickyFaultsOnShuffleboard();
    SmartDashboard.putData("Clear Sticky Faults", new StickyFaultsClear(log));

    // Testing for drivetrain autos and trajectories
    SmartDashboard.putData("Drive Reset SwerveModules", new DriveResetSwerveModules(driveTrain, log));
    SmartDashboard.putData("Zero Gyro", new DriveZeroGyro(driveTrain, log));
    SmartDashboard.putData("Zero Odometry", new DriveResetPose(0, 0, 0, false, driveTrain, log));
    // SmartDashboard.putData("Set Odometry if out of tol", new DriveResetPose(2, 2, 180, true, driveTrain, log));      // For testing only
    SmartDashboard.putData("Drive Reset Pose", new DriveResetPose(driveTrain, log));
    SmartDashboard.putData("Calibrate Drive Motors", new DriveCalibration(0.5, 12, 0.05, driveTrain, log));
    SmartDashboard.putData("Calibrate Turn Motors", new DriveTurnCalibration(1.0, 10, 0.2, driveTrain, log));
    SmartDashboard.putData("Drive Wheels 0 deg", new DriveSetState(0, 0, false, driveTrain, log));
    SmartDashboard.putData("Drive Wheels +85 deg", new DriveSetState(0, 85, false, driveTrain, log));
    SmartDashboard.putData("Drive Wheels +95 deg", new DriveSetState(0, 95, false, driveTrain, log));
    SmartDashboard.putData("Drive 1.5 mps 0 deg", new DriveSetState(1.5, 0, false, driveTrain, log));
    SmartDashboard.putData("Drive Straight", new DriveStraight(false, false, false, driveTrain, log));

    // Testing for autos and trajectories
    SmartDashboard.putData("Drive To Pose", new DriveToPose(driveTrain, log));
    SmartDashboard.putData("Drive To Pose Test", new DriveToPose(new Pose2d(1, 1, Rotation2d.fromDegrees(0)), driveTrain, log));
    SmartDashboard.putData("Drive Trajectory Relative", new DriveTrajectory(CoordType.kRelative, StopType.kBrake, trajectoryCache.cache[TrajectoryType.test.value], driveTrain, log));
    SmartDashboard.putData("Drive Trajectory Curve Relative", new DriveTrajectory(CoordType.kRelative, StopType.kBrake, trajectoryCache.cache[TrajectoryType.testCurve.value], driveTrain, log));
    SmartDashboard.putData("Drive Trajectory Absolute", new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, trajectoryCache.cache[TrajectoryType.test.value], driveTrain, log));  
    SmartDashboard.putData("Example Auto S-Shape", new ExampleAuto(driveTrain));
    SmartDashboard.putData("Drive Trajectory Straight", new DriveTrajectory(
          CoordType.kRelative, StopType.kBrake,
          TrajectoryGenerator.generateTrajectory(
            new Pose2d(0,0,new Rotation2d(0)), 
            List.of(), 
            new Pose2d(1.0,0,new Rotation2d(0)), 
            Constants.TrajectoryConstants.swerveTrajectoryConfig),
          driveTrain, log));
    SmartDashboard.putData("Drive to closest goal", new DriveToPose(() -> field.getInitialColumn(field.getClosestGoal()), driveTrain, log));
  
    //Grabber commands
    SmartDashboard.putData("Grabber Stop", new GrabberStopMotor(grabber, log));
    SmartDashboard.putData("Grabber Pick Up",new GrabberPickUp(grabber, log));
    SmartDashboard.putData("Grabber Eject", new GrabberEject(grabber, log));

    //LED commands
    SmartDashboard.putData("LED Rainbow", new LEDSetPattern(LED.rainbowLibrary, 0, 0.5, led, log));
    SmartDashboard.putData("LED Flash Team Color", new LEDSetPattern(LED.teamFlashingColorsLibrary, 0, 0.5, led, log));
    SmartDashboard.putData("LED Full Team Color", new LEDSetPattern(LED.teamFullColorsLibrary, 0, 0.5, led, log));
    SmartDashboard.putData("LED moving Team Color", new LEDSetPattern(LED.teamMovingColorsLibrary, 0, 0.5, led, log));
    SmartDashboard.putData("LED OFF", new LEDSetStrip("Red", 0, led, log));
    SmartDashboard.putData("LED Yellow", new LEDSetStrip("Yellow", 1, led, log));
    SmartDashboard.putData("LED Purple", new LEDSetStrip("Purple", 1, led, log));

    //Manipulator Commands
    SmartDashboard.putData("Manipulator Stop", new ManipulatorStopMotor(manipulator, log));
    SmartDashboard.putData("Manipulator Pick Up",new ManipulatorSetSpeed(0.5, manipulator, log));
    SmartDashboard.putData("Manipulator Eject", new ManipulatorSetSpeed(-0.5, manipulator, log));
    SmartDashboard.putData("Manipulator Cone", new ManipulatorSetPistonPosition(true, manipulator, log));
    SmartDashboard.putData("Manipulator Cube", new ManipulatorSetPistonPosition(false, manipulator, log));
    SmartDashboard.putData("Manipulator Toggle", new ManipulatorTogglePiston(manipulator, log));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    configureXboxButtons(); // configure xbox controller
    configureJoystickButtons(); // configure joysticks
    configureCopanel(); // configure copanel
  }

  /**
   * Configures XBox buttons and controls
   */
  private void configureXboxButtons(){
    //check povtrigger and axis trigger number bindings
    // Trigger xbPOVUp = new POVTrigger(xboxController, 0);
    // Trigger xbPOVRight = new POVTrigger(xboxController, 90);
    //Trigger xbPOVDown = new POVTrigger(xboxController, 180);
    // Trigger xbPOVLeft = new POVTrigger(xboxController, 270);
    
    // Triggers for all xbox buttons
    Trigger xbLT = xboxController.leftTrigger();
    Trigger xbRT = xboxController.rightTrigger();
    Trigger xbA = xboxController.a();
    // Trigger xbB = xboxController.b();
    // Trigger xbY = xboxController.y();
    // Trigger xbX = xboxController.x();
    // Trigger xbLB = xboxController.leftBumper();
    // Trigger xbRB = xboxController.rightBumper();
    // Trigger xbBack = xboxController.back();
    // Trigger xbStart = xboxController.start();
    // Trigger xbPOVUp = xboxController.povUp();
    // Trigger xbPOVRight = xboxController.povRight();
    // Trigger xbPOVLeft = xboxController.povLeft();
    // Trigger xbPOVDown = xboxController.povDown();
    
    // right trigger 
    xbRT.whileTrue(new GrabberPickUp(grabber, log));
    // xbRT.whenActive(new ShootSequence(uptake, feeder, shooter, log),false);

    // left trigger
    xbLT.whileTrue(new GrabberEject(grabber, log));
    // xbLT.whenActive(new TurretTurnAngleTwice(TargetType.kVisionOnScreen, false, turret, pivisionhub, log));
    // xbLT.whenInactive(new TurretStop(turret, log));

    
    //a
    xbA.whileTrue(new GrabberStopMotor(grabber, log));       
    
    //b
    // xbB.whileTrue(command));         
 
    //y
    // xb[4].whenHeld(new ShootSetup(false, 4100, pivisionhub, shooter, log));        
    
    //x
    // xb[3].whileTrue(new ShootSetup(true, 3100, pivisionhub, shooter, log));        
    
    // LB = 5, RB = 6
    // xb[5].onTrue(new TurretSetPercentOutput(-0.1, turret, log));
    // xb[5].whenReleased(new TurretStop(turret, log));
    // xb[6].onTrue(new TurretSetPercentOutput(+0.1, turret, log));
    // xb[6].whenReleased(new TurretStop(turret, log));

    // back = 7, start = 8 
    // xb[7].whenHeld(new ShootSetup(false, 500, pivisionhub, shooter, log));   // micro shot for use in the pit
    // xb[8].whenHeld(new ClimberSetExtended(false,climber, log)); 

    // pov is the d-pad (up, down, left, right)
    // xbPOVUp.whenActive(new TurretTurnAngle(TargetType.kAbsolute, 0, 2, turret, pivisionhub, log));
    // xbPOVRight.whenActive(new TurretTurnAngle(TargetType.kAbsolute, 45, 2, turret, pivisionhub, log));
    // xbPOVLeft.whenActive(new TurretTurnAngle(TargetType.kAbsolute, -45, 2, turret, pivisionhub, log));
    //xbPOVDown.whenActive(new StopAllMotors(feeder, shooter, intakeFront, uptake, log));
  }

  /**
   * Define drivers joystick button mappings.
   */
  public void configureJoystickButtons() {
    JoystickButton[] left = new JoystickButton[3];
    JoystickButton[] right = new JoystickButton[3];

    for (int i = 1; i < left.length; i++) {
      left[i] = new JoystickButton(leftJoystick, i);
      right[i] = new JoystickButton(rightJoystick, i);
    }

    // TODO Add a button binding to zero the Pose angle (after adding that feature to DriveResetPose).
    // If the robot angle drifts (or is turned on with the wrong facing), then this button can be used to 
    // reset the robot facing for field-oriented control.  Turn the robot so that it is facing away
    // from the driver, then press this button.

    // left joystick left button
    //left[1].onTrue(new IntakeRetractAndFlush(intakeFront, uptake, feeder, log));
    // resets current angle to 0, keeps current X and Y
    left[1].onTrue(new DriveResetPose(0, false, driveTrain, log));
   
    // left joystick right button
    right[1].onTrue(new DriveToPose(CoordType.kAbsolute, 0, driveTrain, log));
    right[2].onTrue(new DriveToPose(CoordType.kRelative, 180, driveTrain, log));

    //left[2].onTrue(new IntakeRetractAndFlush(intakeFront, uptake, feeder, log));
      
    // right joystick left button
    // right[1].onTrue(new IntakeExtendAndTurnOnMotors(intakeFront, uptake, log)); 

    // right joystick right button
    // right[2].onTrue(new IntakeRetractAndFlush(intakeFront, uptake, feeder, log));  
  }

  /** 
   * Define Copanel button mappings.
   *  
   *  1  3  5  8
   *  2  4  6  8
   *      
   *  9  11 13 7
   *  10 12 14 7
   * 
   *  15
   *  16
   */
  public void configureCopanel() {
    JoystickButton[] coP = new JoystickButton[20];

    for (int i = 1; i < coP.length; i++) {
      coP[i] = new JoystickButton(coPanel, i);
    }

    // top row UP then DOWN, from LEFT to RIGHT
    // coP[1].onTrue(new IntakeToColorSensor(intakeFront, uptake, log)); 
    // coP[2].onTrue(new ShooterSetVelocity(InputMode.kSpeedRPM, ShooterConstants.shooterDefaultRPM, shooter, log)); 

    // coP[3].onTrue(new UptakeFeedBall(uptake, feeder, log)); 
    // coP[4].onTrue(new UptakeEjectBall(uptake, log)); 

    // coP[5].whenHeld(new ClimbSetPercentOutput(0.4, climb, log)); // manually raise climb arms, slowly
    // coP[6].whenHeld(new ClimbSetPercentOutput(-0.4, climb, log)); // manually lower climb arms, slowly
    
    // top row RED SWITCH
    // coP[8].onTrue(new StopAllMotors(feeder, shooter, intakeFront, uptake, log));

    // middle row UP then DOWN, from LEFT to RIGHT
    // coP[9].onTrue(new IntakeSetPercentOutput(IntakeConstants.onPct, IntakeConstants.onPctTransfer, intakeFront, log)); // forward intake and transfer
    // coP[10].onTrue(new IntakeSetPercentOutput(-IntakeConstants.onPct, -IntakeConstants.onPctTransfer, intakeFront, log)); // reverse intake and transfer

    // coP[11].onTrue(new UptakeSetPercentOutput(-UptakeConstants.onPct, 0, uptake, log)); // reverse uptake
    // coP[12].onTrue(new UptakeSetPercentOutput(UptakeConstants.onPct, 0, uptake, log)); // forward uptake

    // coP[13].onTrue(new FeederSetPercentOutput(-FeederConstants.onPct, feeder, log)); // reverse feeder
    // coP[14].onTrue(new FeederSetPercentOutput(FeederConstants.onPct, feeder, log)); // forward feeder

    // middle row UP OR DOWN, fourth button
    // coP[7].onTrue(new IntakePistonToggle(intakeFront, uptake, log)); 

    // bottom row UP then DOWN, from LEFT to RIGHT
    // coP[15].onTrue(new ClimberSetExtended(true,climber, log)); // climb extend
    // coP[16].onTrue(new ClimberSetExtended(false,climber, log)); // climb retract
  }


  /**
   * Sets the rumble on the XBox controller
   * @param percentRumble The normalized value (0 to 1) to set the rumble to
   */
	public void setXBoxRumble(double percentRumble) {
		xboxController.getHID().setRumble(RumbleType.kLeftRumble, percentRumble);
    xboxController.getHID().setRumble(RumbleType.kRightRumble, percentRumble);

    if (percentRumble == 0) rumbling = false;
    else rumbling = true;
  }
  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoSelection.getAutoCommand(driveTrain, log);
  }


  /**
   * Method called when robot is initialized.
   */
  public void robotInit() {
    SmartDashboard.putBoolean("RobotPrefs Initialized", RobotPreferences.prefsExist());
    if(!RobotPreferences.prefsExist()) {
      RobotPreferences.recordStickyFaults("RobotPreferences", log);
    }

    compressor.disable();
    // compressor.enableDigital();

    // Set initial robot position on field
    // This takes place a while after the drivetrain is created, so after any CanBus delays.
    driveTrain.resetPose(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0)));  
  }

  /**
   * robotPeriodic is run every 20msec
   */
  public void robotPeriodic(){
    log.advanceLogRotation();
    allianceSelection.periodic();
  }

  /**
   * Method called when robot is disabled.
   */
  public void disabledInit() {
    log.writeLogEcho(true, "Disabled", "Robot disabled");   // Don't log the word "Init" here -- it affects the Excel macro

    driveTrain.setDriveModeCoast(true);     // When pushing a disabled robot by hand, it is a lot easier to push in Coast mode!!!!
    driveTrain.stopMotors();                // SAFETY:  Turn off any closed loop control that may be running, so the robot does not move when re-enabled.

    patternTeamMoving.schedule();
  }

  /**
   * Method called once every scheduler cycle when robot is disabled.
   */
  public void disabledPeriodic() {
    // Check for CAN bus error.  This is to prevent the issue that caused us to be eliminated in 2020!
    if (driveTrain.canBusError()) {
      RobotPreferences.recordStickyFaults("CAN Bus", log);
    }  //    TODO May want to flash this to the driver with some obvious signal!
    // boolean error = true;  
    // if (error == false) {
    //   if(!patternTeamMoving.isScheduled()) patternTeamMoving.schedule();
    // }
    // else {
    //   patternTeamMoving.cancel();
    //   led.setStrip("Red", 0.5, 0);
    // }
  }
  
  /**
   * Method called when auto mode is initialized/enabled.
   */
  public void autonomousInit() {
    log.writeLogEcho(true, "Auto", "Mode Init");

    driveTrain.setDriveModeCoast(false);

    if (patternTeamMoving.isScheduled()) patternTeamMoving.cancel();
    if (allianceSelection.getAlliance() == Alliance.Blue) {
      led.setStrip("Blue", 0);
    } else {
      led.setStrip("Red", 0);
    }
    // NOTE:  Do NOT reset the gyro or encoder here!!!!!
    // The first command in auto mode initializes before this code is run, and
    // it will read the gyro/encoder before the reset goes into effect.
  }

  /**
   * Method called once every scheduler cycle when auto mode is initialized/enabled
   */
  public void autonomousPeriodic() {
  }

  /**
   * Method called when teleop mode is initialized/enabled.
   */
  public void teleopInit() {
    log.writeLogEcho(true, "Teleop", "Mode Init");

    driveTrain.setDriveModeCoast(false);

    if (patternTeamMoving.isScheduled()) patternTeamMoving.cancel();
    led.setStrip("Orange", 0);
  }

  /**
   * Method called once every scheduler cycle when teleop mode is initialized/enabled.
   */
  public void teleopPeriodic() {

  }
}
