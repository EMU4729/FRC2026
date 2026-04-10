package frc.robot.constants;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.LinearVelocity;

public class TurretConstants {
    public static final int SHOOTER_MOTOR_1_CANID = 12;
    public static final int SHOOTER_MOTOR_2_CANID = 13;
    public static final int SHOOTER_MOTOR_3_CANID = 14;

    //public static final int MIN_MOTOR_SPEED = 200; // TO-DO
    public static final LinearVelocity ShooterIdleSpeed = MetersPerSecond.of(5); //a <100% speed to idle the shooter wheels at when not shooting //TODO
    public static final double ROTATOR_P = 1;//TODO
    public static final double ROTATOR_I = 0;//TODO
    public static final double ROTATOR_D = 0;//TODO
    public static final double rotatorMotorRatio = 1;
    public static final double HOOD_P = 25;
    public static final double HOOD_I = 8;
    public static final double HOOD_D = 0;
    public static final double hoodMotorRatio = 15*(20/18);

    
}