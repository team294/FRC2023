package frc.robot.utilities;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class Motor<T> {
    public T controller;

    public Motor(int id) {
        
    }

    private static class WPI_TalonFXMotor extends Motor<WPI_TalonFX> {
        public WPI_TalonFXMotor(int id) {
            super(id);
        }
    }

    public static Motor<WPI_TalonFX> createTalonFX(int id) {
        return new WPI_TalonFXMotor(id);
    }
}
