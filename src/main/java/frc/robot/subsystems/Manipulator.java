// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import static frc.robot.utilities.StringUtil.*;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Ports;
import frc.robot.utilities.FileLog;
import frc.robot.utilities.Loggable;

public class Manipulator extends SubsystemBase implements Loggable {
  /** Creates a new Manipulator. */
  private final FileLog log;
  private int logRotationKey;         // key for the logging cycle for this subsystem
  private boolean fastLogging = false; 

  private String subsystemName;

  private final CANSparkMax motor;
  private final DoubleSolenoid pneumaticDoubleSolenoid;

  private boolean pistonExtended = false;

  /**
   * Constructs the Manipulator subsystem
   * @param subsystemName  String name for subsystem
   * @param inverted inverts the motor, true inverts motor
   * @param solenoidForwardChannel
   * @param solenoidReverseChannel
   * @param log object for logging
   */
  public Manipulator(String subsystemName, boolean inverted, int solenoidForwardChannel, int solenoidReverseChannel, FileLog log) {
    motor = new CANSparkMax(Ports.CANManipulator, MotorType.kBrushless);
    pneumaticDoubleSolenoid = new DoubleSolenoid(Ports.CANPneumaticHub, PneumaticsModuleType.REVPH, solenoidForwardChannel, solenoidReverseChannel);
    this.subsystemName = subsystemName;
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

  /**
   * Sets the voltage of the motor
   * @param voltage voltage
   */
  public void setVoltage(double voltage){
    motor.setVoltage(voltage);
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
    motor.stopMotor();
  }

  /**
   * @return stator current of the motor in amps
   */
  public double getAmps(){
    return motor.getStatorCurrent();
  }

  /**
   * Sets if the piston should be extended or not
   * 
   * @param extend true = extend, false = retract
   */
  public void setPistonExtended(boolean extend) {
    pistonExtended = extend;
    pneumaticDoubleSolenoid.set(extend ? Value.kForward : Value.kReverse);
  }

  /**
   * Returns if intake piston is extended or not
   * @return true = extended, false = retracted
   */
  public boolean getPistonExtended() {
    return pistonExtended;
  }

  /**
   * Toggles the piston between cone and cube
   */
  public void togglePistonExtended(){
    log.writeLog(false, subsystemName, "togglePiston", "from extended", getPistonExtended());
    setPistonExtended(!getPistonExtended());
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
      SmartDashboard.putNumber(buildString(subsystemName, "Amps"), motor.getStatorCurrent());
      SmartDashboard.putNumber(buildString(subsystemName, "Bus Volt"), motor.getBusVoltage());
      SmartDashboard.putNumber(buildString(subsystemName, "Volt"), motor.getMotorOutputVoltage());
      SmartDashboard.putNumber(buildString(subsystemName, "Out Percent"), motor.getMotorOutputPercent());
      SmartDashboard.putNumber(buildString(subsystemName, "Out Temperature"), motor.getTemperature());
      SmartDashboard.putBoolean(buildString(subsystemName, "Piston extend"), pistonExtended);
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
    "Volt", motor.getMotorOutputVoltage(),
    "Amps", motor.getStatorCurrent(),
    "Temperature", motor.getTemperature(),
    "Piston extended", getPistonExtended()
    );
  }

  
}
