package frc.robot.commands.Turret;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.OI;
import frc.robot.Subsystems;
import frc.robot.subsystems.IntakeSub;

/**
 * Simple intake command — runs the intake at the given speed while active.
 * Mirrors behaviour of the top-level ActivateIntakeCommand but lives in the
 * Turret commands package (keeps compatibility with existing button wiring if used).
 */
public class IntakeCommand extends Command {
	private static LinearVelocity speed;
			private final static IntakeSub intake = Subsystems.intake;
				public static final double MOTOR_SPEED = 1.0;
			
				public IntakeCommand(LinearVelocity speed) {
					IntakeCommand.speed = speed;
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
			// Add this factory method to IntakeCommand, or make a separate AutoIntakeCommand
		
		
			@Override
			public void end(boolean interrupted) {
				intake.setSpeed(MetersPerSecond.of(0));
			}
			public static IntakeCommand forAuto() {
			return new IntakeCommand(MetersPerSecond.of(MOTOR_SPEED)) {
				@Override
				public void execute() {
					intake.setSpeed(speed); // no button check
        }
    };
}

public static Command forAutoOff() {
    return new InstantCommand(() -> intake.setSpeed(MetersPerSecond.of(0)), intake);
}
}



