package frc.robot.constants;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.LinearVelocity;

public class HopperConstants {
    // CAN ID for the single hopper motor. Verify on robot and change as needed.
    public static final int HOPPER_MOTOR_CANID = 6; // TODO: set correct CAN ID

    // Geometry / kinematics (placeholders — update to match hardware)
    public static final double HOPPER_WHEEL_RADIUS_M = 0.01; // meters
    public static final double HOPPER_GEAR_RATIO = 1.0; // motor rotations -> output

    // Default operating speed for the hopper (linear m/s). Use small value for extension.
    public static final LinearVelocity HOPPER_DEFAULT_SPEED = MetersPerSecond.of(0.05);
}
