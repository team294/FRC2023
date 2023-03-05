// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import static frc.robot.utilities.StringUtil.*;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ports;
import frc.robot.utilities.FileLog;
import frc.robot.utilities.Loggable;

public class Intake extends SubsystemBase implements Loggable {
  /** Creates a new Intake. */
  private final FileLog log;
  private final int logRotationKey;         // key for the logging cycle for this subsystem
  private boolean fastLogging = false; 

  private String subsystemName;

  private final WPI_TalonSRX motor;
  private final DoubleSolenoid pneumaticDoubleSolenoid;

  private boolean pistonExtended = false;     // Default state = retracted

  /**
   * Constructs the Intake subsystem, including rollers and a solenoid to change between deploy and stowed.
   * @param subsystemName  String name for subsystem
   * @param inverted inverts the motor, true inverts motor
   * @param solenoidForwardChannel
   * @param solenoidReverseChannel
   * @param log object for logging
   */
  public Intake(FileLog log) {
    motor = new WPI_TalonSRX(Ports.CANIntake);
    pneumaticDoubleSolenoid = new DoubleSolenoid(Ports.CANPneumaticHub, PneumaticsModuleType.REVPH, Ports.SolIntakeFwd, Ports.SolIntakeRev);
    subsystemName = "Intake";
    this.log = log;
    
    logRotationKey = log.allocateLogRotation();

    motor.setNeutralMode(NeutralMode.Coast);
    motor.setInverted(false);
    motor.configVoltageCompSaturation(12.0, 100);
    motor.enableVoltageCompensation(true);
    motor.configOpenloopRamp(0.3, 100);     //seconds from neutral to full
  }

    /**
   * Returns the name of the subsystem
   */
  public String getName(){
    return subsystemName;
  }

  /**
   * Sets the percent of the motor, + is intake, - is outtake
   * @param percent -1.0 to +1.0
   */
  public void setMotorPercentOutput(double percent){
    motor.set(percent);
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
   * @param extend true = deploy, false = retract
   */
  public void setDeployed(boolean extend) {
    pistonExtended = extend;
    pneumaticDoubleSolenoid.set(extend ? Value.kForward : Value.kReverse);
  }

  /**
   * Returns if intake piston is extended or not
   * @return true = deployed, false = retracted
   */
  public boolean isDeployed() {
    return pistonExtended;
  }

  /**
   * Toggles the piston between deploy and undeployed
   */
  public void toggleDeploy(){
    log.writeLog(false, subsystemName, "togglePiston", "from extended", isDeployed());
    setDeployed(!isDeployed());
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
      updateIntakeLog(false);
      // Update data on SmartDashboard
      SmartDashboard.putNumber(buildString(subsystemName, "Amps"), getAmps());
      SmartDashboard.putNumber(buildString(subsystemName, "Bus Volt"), motor.getBusVoltage());
      SmartDashboard.putNumber(buildString(subsystemName, "Out Percent"), motor.getMotorOutputPercent());
      SmartDashboard.putBoolean(buildString(subsystemName, "Piston extend"), pistonExtended);
    }
  }

  /**
   * Writes information about the Intake to the filelog
   * @param logWhenDisabled true will log when disabled, false will discard the string
   */
  public void updateIntakeLog(boolean logWhenDisabled){
    log.writeLog(logWhenDisabled, subsystemName, "Update Variables",
    "Bus Volt", motor.getBusVoltage(),
    "Out Percent", motor.getMotorOutputPercent(),
    "Amps", getAmps(),
    "Piston extended", isDeployed()
    );
  }

  
}
