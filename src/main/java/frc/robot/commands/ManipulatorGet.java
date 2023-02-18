// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;
public class ManipulatorGet extends CommandBase {
    private Manipulator Manipulator;
    private int BehaviorType=0;
    private double MotorPercent=0.0;
    private double ampSensitivity = 0.0;
    private boolean coneGrab=false;
    private boolean cubeGrab=false;

    /**
   * Command with 5 different behaviors for the manipulator (cone-cube grabber)
   *-immediately end command, motor stays at set speed
    -command stays running forever, stop when interrupted
    -wait for cone, then stop motor
    -wait for cube, then stop motor
    -wait for cone or cube then stop motor
   * @param Manipulator [Manipulator] manipulator object
   * @param MotorPercent [Double] percent motors will be run at when command is initialized
   * @param BehaviorType [Enum] that defines behavior of manipualotr (0-4)
   * @param ampSensitivity [Double] minimum value sensitivity must change by to detect that manipulator has picked up object
   */
  public ManipulatorGet(Manipulator Manipulator,double MotorPercent,Enum BehaviorType,double ampSensitivity) {
    this.Manipulator=Manipulator;
    this.MotorPercent=MotorPercent;
    this.BehaviorType=BehaviorType.hashCode();
    this.ampSensitivity=ampSensitivity;
  }

  // Called when the command is initially scheduled.
 
  /*sets motor percent to parameter passed in constructor
   if behavior is set to coneGrab then toggle manipulator to grab cones, vice versa for cubes
   */

   @Override
  public void initialize() {
    Manipulator.setMotorPercentOutput(MotorPercent);
    if(this.BehaviorType==2){
        if(!Manipulator.getPistonCone())
        Manipulator.setPistonCone(true);
    }else if(this.BehaviorType==3){
        if(Manipulator.getPistonCone())
        Manipulator.setPistonCone(false);
    }
  }

  //Called in a loop every time the command is executed

  //checks for amp spike greater than ampSensitivity parameter, toggles coneGrab and cubeGrab booleans depending on the configuration of the manipulator
  @Override
  public void execute(){
    if(Manipulator.getAmps()>ampSensitivity){
        this.coneGrab=this.cubeGrab=false;
        if(Manipulator.getPistonCone()){
            this.coneGrab=true;
        }else{
            this.cubeGrab=true;
        }
    }
    

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
    switch (this.BehaviorType) {
      case 0:
      return true;  
      case 1:
      return false;
      case 2:
        if(coneGrab)
          Manipulator.setMotorPercentOutput(0);
        return coneGrab;
      case 3:
        if(cubeGrab)
          Manipulator.setMotorPercentOutput(0);
        return cubeGrab;
      case 4:
        if(cubeGrab||coneGrab)
          Manipulator.setMotorPercentOutput(0);
        return (coneGrab||cubeGrab);
  
      default:
        Manipulator.setMotorPercentOutput(0);
        return true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Manipulator.setMotorPercentOutput(0);
  }

}
