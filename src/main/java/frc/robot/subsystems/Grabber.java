// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.FileLog;

public class Grabber extends SubsystemBase {
  /** Creates a new Grabber. */
  private final FileLog log;
  private final WPI_TalonFX motor;
  private String swName;
  
  public Grabber(int CANPort, String subsystemName, FileLog log) {
    motor = new WPI_TalonFX(CANPort);
    this.log = log;
    swName = subsystemName;
  }

  public void setMotorPercentOutput(double percent){
    motor.set(ControlMode.PercentOutput, percent);
  }

  public void stopMotor(){
    motor.stopMotor();
  }

  public double getAmps(){
    return motor.getStatorCurrent();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
