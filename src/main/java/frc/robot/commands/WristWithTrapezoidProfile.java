// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.Constants;
// import frc.robot.RobotContainer;
// import frc.robot.Constants.WristConstants;
// import frc.robot.subsystems.Wrist;
// import frc.robot.utilities.FileLog;
// import frc.robot.utilities.TrapezoidProfileBCR;

// public class WristWithTrapezoidProfile extends CommandBase {
//   private final Timer timer = new Timer();
//   private FileLog log;
//   private TrapezoidProfileBCR tProfile; // wpilib trapezoid profile generator
//   private TrapezoidProfileBCR.State tStateCurr; // initial state of the system (position in deg and time in sec)
//   private TrapezoidProfileBCR.State tStateNext; // next state of the system as calculated by the profile generator
//   private TrapezoidProfileBCR.State tStateFinal; // goal state of the system (position in deg and time in sec)
//   private TrapezoidProfileBCR.Constraints tConstraints; // max vel (deg/sec) and max accel (deg/sec/sec) of the system
//   private double initialAngle;
//   private double goalAngle;
//   private Wrist wrist;

//   /** Creates a new TrapezoidTemplate. */
//   public WristWithTrapezoidProfile(double goalAngle, Wrist wrist, FileLog log) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     this.log = log;
//     this.wrist = wrist;
//     this.goalAngle = goalAngle;
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     timer.reset();
//     timer.start();
//     initialAngle = wrist.getWristAngle();
//     tStateFinal = new TrapezoidProfileBCR.State(goalAngle, 0.0); // initialize goal state (degrees to turn)
//     tStateCurr = new TrapezoidProfileBCR.State(initialAngle, 0.0); // initialize initial state (relative turning, so assume initPos is 0 degrees)
//     tConstraints = new TrapezoidProfileBCR.Constraints(WristConstants.kMaxAngularVel, WristConstants.kMaxAngularAcc); // initialize velocity and accel limits
//     tProfile = new TrapezoidProfileBCR(tConstraints, tStateFinal, tStateCurr); // generate profile
    

//     addRequirements(wrist);

//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     double curTime = timer.get();

    
//     tStateNext = tProfile.calculate(curTime + 0.01);
//     double targetVel = tStateNext.velocity;                                                                                                             


//     wrist.setWristVelocity(targetVel);
//     //TrapezoidProfileBCR.State desiredState = tProfile.calculate(curTime);

//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
