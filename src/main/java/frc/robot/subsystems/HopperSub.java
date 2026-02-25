package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.HopperConstants;

/**
 * Hopper subsystem: controls a single motor that expands/contracts the hopper.
 * Implementation follows the style of IntakeSub/TurretFeederSub with a VelocityVoltage controller
 * and a small sim helper.
 */
public class HopperSub extends SubsystemBase {
	private final TalonFX motor = new TalonFX(HopperConstants.HOPPER_MOTOR_CANID);
	private final VelocityVoltage controller = new VelocityVoltage(0).withSlot(0);

	// simple sim variables
	private final TalonFXSimState motorSim;
	private double simSpeed = 0;
	private double simSpeedTarget = 0;
	private final double simAccel = 0.5;

	// Whether the hopper mechanism is currently activated (expanded). Useful for simulation/UI.
	private boolean activated = false;

	public HopperSub() {
		TalonFXConfiguration cfg = new TalonFXConfiguration();
		cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		cfg.Feedback.SensorToMechanismRatio = 1;
		cfg.Slot0.kP = 1;
		cfg.Slot0.kI = 0;
		cfg.Slot0.kD = 0;
		cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

		motor.getConfigurator().apply(cfg);

		motorSim = motor.getSimState();
	}

	/**
	 * Set hopper motor speed using a linear velocity (meters per second) for consistency with other subsystems.
	 * Internally converted to an angular velocity for the VelocityVoltage controller.
	 */
	public void setSpeed(LinearVelocity speed) {
		// Convert linear speed to angular for the controller using constants.
		double radius = HopperConstants.HOPPER_WHEEL_RADIUS_M;
		double ratio = HopperConstants.HOPPER_GEAR_RATIO;

		AngularVelocity aSpeed = RadiansPerSecond.of(speed.in(MetersPerSecond) * ratio / radius);

		motor.setControl(controller.withVelocity(aSpeed));

		if (Robot.isSimulation()) {
			simSpeedTarget = aSpeed.in(RadiansPerSecond);
		}
	}

	public void stop() {
		motor.stopMotor();
		activated = false;
	}

	public void setActivated(boolean on) {
		activated = on;
		if (!on) stop();
	}

	public boolean isActivated() {
		return activated;
	}

	@Override
	public void periodic() {
		// expose a basic telemetry value
		double rpm = motor.getVelocity().getValue().in(RadiansPerSecond);
		SmartDashboard.putNumber("Hopper/motorSpeedRPS", rpm);
		SmartDashboard.putBoolean("Hopper/Activated", activated);
	}

	@Override
	public void simulationPeriodic() {
		if (simSpeed > simSpeedTarget) {
			simSpeed -= simAccel;
		} else if (simSpeed < simSpeedTarget) {
			simSpeed += simAccel;
		}

		motorSim.setRotorVelocity(simSpeed);
	}
}
