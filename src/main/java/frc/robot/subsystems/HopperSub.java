package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.HopperConstants;

/**
 * Hopper subsystem: controls a single motor that expands/contracts the hopper.
 * Implementation follows the style of IntakeSub/TurretFeederSub with a VelocityVoltage controller
 * and a small sim helper.
 */
public class HopperSub extends SubsystemBase {
	private final WPI_VictorSPX motor = new WPI_VictorSPX(HopperConstants.HOPPER_MOTOR_CANID);

	// simple sim variables
	// private final TalonFXSimState motorSim;
	// private double simSpeed = 0;
	// private double simSpeedTarget = 0;
	// private final double simAccel = 0.5;

	public HopperSub() {
		// TalonFXConfiguration cfg = new TalonFXConfiguration();
		// cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		// cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
		// motor.getConfigurator().apply(cfg);
		motor.setInverted(true);
		motor.setNeutralMode(NeutralMode.Brake);
	}

	public void setDutyCycle(double dutyCycle) {
		motor.set(dutyCycle);
	}

	public Command runCommand(double dutyCycle) {
		return this.startEnd(() -> setDutyCycle(dutyCycle), () -> setDutyCycle(0));
	}

	@Override
	public void simulationPeriodic() {
		// if (simSpeed > simSpeedTarget) {
		// 	simSpeed -= simAccel;
		// } else if (simSpeed < simSpeedTarget) {
		// 	simSpeed += simAccel;
		// }

		// motorSim.setRotorVelocity(simSpeed);
	}
}
