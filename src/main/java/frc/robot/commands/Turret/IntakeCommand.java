package frc.robot.commands.Turret;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.Subsystems;
import frc.robot.subsystems.IntakeSub;

/**
 * Simple intake command — runs the intake at the given speed while active.
 * Mirrors behaviour of the top-level ActivateIntakeCommand but lives in the
 * Turret commands package (keeps compatibility with existing button wiring if used).
 */
public class IntakeCommand extends Command {
	private final LinearVelocity speed;
	private final IntakeSub intake = Subsystems.intake;
	public static final double MOTOR_SPEED = 1.0;

	public IntakeCommand(LinearVelocity speed) {
		this.speed = speed;
		addRequirements(intake);

	}

	@Override
	public void initialize() {
		intake.setSpeed(MetersPerSecond.of(0));
	}

	@Override
	public void execute() {
		
        if (OI.pilot.y().getAsBoolean()){
            intake.setSpeed(speed);
        }

	}

	@Override
	public void end(boolean interrupted) {
		intake.setSpeed(MetersPerSecond.of(0));
	}
}
