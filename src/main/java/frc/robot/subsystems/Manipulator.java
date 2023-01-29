// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ports;
import frc.robot.utilities.FileLog;

public class Manipulator extends SubsystemBase {
  /** Creates a new Manipulator. */
  private final FileLog log;
  private int logRotationKey;         // key for the logging cycle for this subsystem
  private boolean fastLogging = false; 
  private boolean inverted;

  private String subsystemName;

  private final WPI_TalonFX motor;
  private final DoubleSolenoid pneumaticDoubleSolenoid;

  /**
   * Constructs the Manipulator subsystem
   * @param inverted inverts the motor
   * @param log object for logging
   */
  public Manipulator(String subsystemName, boolean inverted, FileLog log) {
    motor = new WPI_TalonFX(Ports.CANManipulator);
    pneumaticDoubleSolenoid = new DoubleSolenoid(null, logRotationKey, logRotationKey);

    this.subsystemName = subsystemName;
    this.inverted = inverted;
    this.log = log;
    
    logRotationKey = log.allocateLogRotation();

    motor.configFactoryDefault();
    motor.setInverted(inverted);
    motor.setNeutralMode(NeutralMode.Coast);
    motor.configPeakOutputForward(1.0);
    motor.configPeakOutputReverse(-1.0);
    motor.configNeutralDeadband(0.01);
    motor.configVoltageCompSaturation(12);
    motor.enableVoltageCompensation(true);
    motor.configOpenloopRamp(0.05);   //seconds from neutral to full
    motor.configClosedloopRamp(0.05); //seconds from neutral to full
  

  }

    /**
   * Returns the name of the subsystem
   */
  public String getName(){
    return subsystemName;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
