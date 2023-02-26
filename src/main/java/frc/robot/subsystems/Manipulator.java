// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ports;
import frc.robot.utilities.FileLog;
import frc.robot.utilities.Loggable;

public class Manipulator extends SubsystemBase implements Loggable {
  /** Creates a new Manipulator. */
  private final FileLog log;
  private int logRotationKey;         // key for the logging cycle for this subsystem
  private boolean fastLogging = false; 

  private String subsystemName = "Manipulator";

  private final TalonSRX motor;
  private final DoubleSolenoid pneumaticDoubleSolenoid;

  private boolean pistonCone = true;     // manipulator piston position (true = cone, false = cube) // TODO set default state

  /**
   * Constructs the Manipulator subsystem, including rollers and a solenoid to change between cube and cone configuration.
   * @param log object for logging
   */
  public Manipulator(FileLog log) {
    motor = new TalonSRX(Ports.CANManipulator);
    pneumaticDoubleSolenoid = new DoubleSolenoid(Ports.CANPneumaticHub, PneumaticsModuleType.REVPH, Ports.SolManipulatorFwd, Ports.SolManipulatorRev);
    this.log = log;
    
    logRotationKey = log.allocateLogRotation();

    // motor.restoreFactoryDefaults();
    // motor.setIdleMode(IdleMode.kCoast);
    // motor.setInverted(false);       // TODO determine if needs to be inverted
    // motor.enableVoltageCompensation(12);
    // motor.setOpenLoopRampRate(0.05);    //seconds from neutral to full
    // motor.setClosedLoopRampRate(0.05);  //seconds from neutral to full
  }

  /**
   * Returns the name of the subsystem
   */
  public String getName(){
    return subsystemName;
  }

  /**
   * Sets the percent of the motor
   * @param percent
   */
  public void setMotorPercentOutput(double percent){
    motor.set(ControlMode.PercentOutput, percent);
  }

  /**
   * Stops the motor
   */
  public void stopMotor(){
    motor.set(ControlMode.PercentOutput, 0);
  }

  /**
   * @return stator current of the motor in amps
   */
  public double getAmps(){
    return motor.getStatorCurrent();
  }

  /**
   * Sets if the piston should be in cone or cube position
   * 
   * @param cone true = cone, false = cube
   */
  public void setPistonCone(boolean cone) {
    pistonCone = cone;
    pneumaticDoubleSolenoid.set(cone ? Value.kForward : Value.kReverse);
  }

  /**
   * Returns if manipulator piston is set for cone position
   * @return true = cone, false = cube
   */
  public boolean getPistonCone() {
    return pistonCone;
  }

  /**
   * Toggles the piston between cone and cube
   */
  public void togglePiston(){
    log.writeLog(false, subsystemName, "togglePiston", "from position is cone", getPistonCone());
    setPistonCone(!getPistonCone());
  }

  

  // ************ Information methods

  /**
   * Turns file logging on every scheduler cycle (~20ms) or every 10 cycles (~0.2 sec)
   * @param enabled true = every cycle, false = every 10 cycles
   */ 
  @Override
  public void enableFastLogging(boolean enabled) {
    fastLogging = enabled;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(fastLogging || log.isMyLogRotation(logRotationKey)) {
      updateManipulatorLog(false);
      // Update data on SmartDashboard
      SmartDashboard.putNumber("Manipulator Amps", motor.getOutputCurrent());
      SmartDashboard.putNumber("Manipulator Bus Volt", motor.getBusVoltage());
      SmartDashboard.putNumber("Manipulator Out Percent", motor.getMotorOutputPercent());
      SmartDashboard.putNumber("Manipulator Temperature", motor.getTemperature());
      SmartDashboard.putBoolean("Manipulator Cone", pistonCone);
    }
  }

  /**
   * Writes information about the Manipulator to the filelog
   * @param logWhenDisabled true will log when disabled, false will discard the string
   */
  public void updateManipulatorLog(boolean logWhenDisabled){
    log.writeLog(logWhenDisabled, subsystemName, "Update Variables",
    "Bus Volt", motor.getBusVoltage(),
    "Out Percent", motor.getMotorOutputPercent(),
    "Amps", motor.getStatorCurrent(),
    "Temperature", motor.getTemperature(),
    "Piston cone", getPistonCone()
    );
  }

  
}
