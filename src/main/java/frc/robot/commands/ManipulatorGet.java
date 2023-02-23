// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;
import frc.robot.utilities.FileLog;
public class ManipulatorGet extends CommandBase {

    public enum BehaviorType{
      immediatelyEnd,
      runForever, 
      waitForCone,
      waitForCube,
      waitForConeOrCube
    }

    private final Manipulator manipulator;
    private final FileLog log;

    private double motorPercent = 0.0;
    private double ampSensitivity = 0.0;
    //false is cube, true is cone
    //private Boolean grabType = null;
    private boolean coneGrab;
    private boolean cubeGrab;
    private BehaviorType behaviorType;

    

    /**
   * Command with 5 different behaviors for the manipulator (cone-cube grabber)
   *immdiatelyEnd = immediately end command, motor stays at set speed
    runForever = command stays running forever, stop when interrupted
    waitForCone = wait for cone, then stop motor
    waitForCube = wait for cube, then stop motor
    waitForConeOrCube = wait for cone or cube then stop motor
   * @param Manipulator [Manipulator] manipulator object
   * @param MotorPercent [Double] percent motors will be run at when command is initialized
   * @param BehaviorType [BehaviorType] enum that defines behavior of manipulator (0-4)
   * @param ampSensitivity [Double] minimum value sensitivity must change by to detect that manipulator has picked up object
   */
  public ManipulatorGet(double motorPercent, double ampSensitivity, BehaviorType behaviorType,  Manipulator manipulator, FileLog log) {
    this.manipulator = manipulator;
    this.log = log;
    this.motorPercent = motorPercent;
    this.behaviorType = behaviorType;
    this.ampSensitivity = ampSensitivity;

    addRequirements(manipulator);
  }

  // Called when the command is initially scheduled.
 
  /*sets motor percent to parameter passed in constructor
   if behavior is set to coneGrab then toggle manipulator to grab cones, vice versa for cubes
   */

   @Override
  public void initialize() {
    //grabType = null;
    cubeGrab = false;
    coneGrab = false;
    manipulator.setMotorPercentOutput(motorPercent);

    if(behaviorType == BehaviorType.waitForCone){
      manipulator.setPistonCone(true);

    } else if(behaviorType == BehaviorType.waitForCube){
      manipulator.setPistonCone(false);

    }
    log.writeLog(false, "ManipulatorGet", "Start");


  }

  //Called in a loop every time the command is executed

  //checks for amp spike greater than ampSensitivity parameter, toggles coneGrab and cubeGrab booleans depending on the configuration of the manipulator
  @Override
  public void execute(){
    //using amps
    if(manipulator.getAmps() > ampSensitivity){

        if(manipulator.getPistonCone()){
          //grabType = true;
          coneGrab = true;
        } else {
          //grabType = false;
          cubeGrab = true;
        }
    }
    //using sensor
    
    log.writeLog(false, "ManipulatorGet", "Amps", manipulator.getAmps(), "Motor Percent Output", motorPercent);
    

  }

  /*end condition behaviors for finished command
    
    -immediately end command, motor stays at set speed
    -command stays running forever, stop when interrupted
    -wait for cone, then stop motor
    -wait for cube, then stop motor
    -wait for cone or cube then stop motor
   */
  @Override
  public boolean isFinished(){
    //boolean notNull = grabType != null;

    switch (behaviorType) {
      case immediatelyEnd:
        return true;  
      case runForever:
        return false;
      case waitForCone:
        if(coneGrab)
          manipulator.stopMotor();
        return (coneGrab);
      case waitForCube:
        if(cubeGrab)
          manipulator.stopMotor();
        return (cubeGrab);
      case waitForConeOrCube:
        if(cubeGrab || coneGrab)
          manipulator.stopMotor();
        return cubeGrab || coneGrab;
  
      default:
        manipulator.stopMotor();;
        return true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    manipulator.stopMotor();
  }

}
