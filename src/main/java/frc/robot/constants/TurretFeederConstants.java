package frc.robot.constants;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.LinearVelocity;

public class TurretFeederConstants {
    public static final int FEEDER_MOTOR_CANID = 16;
    public static final LinearVelocity FEEDER_SPEED = MetersPerSecond.of(100); //TO-DO
    public static final int TURRENT_MOTOR_1_CANID = 35; //TODO change this

    public static final LinearVelocity TARGET_SPEED = MetersPerSecond.of(1);
}