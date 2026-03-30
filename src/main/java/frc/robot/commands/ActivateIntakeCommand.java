package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;
import frc.robot.subsystems.IntakeSub;

public class ActivateIntakeCommand extends Command {

    public static final double MOTOR_SPEED = 1.0;
    public static final double AUTO_INTAKE_SECONDS = 2.0;

    private final IntakeSub intake = Subsystems.intake;
    private final LinearVelocity speed;

    private final Timer timeOutIntake;

    /**
     * Creates an ActivateIntakeCommand with a given speed.
     * Used for both teleop and auto.
     *
     * @param speed The linear speed to run the intake wheels at.
     */
    public ActivateIntakeCommand(LinearVelocity speed) {
        timeOutIntake = new Timer();
        this.speed = speed;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setExtendAngle();
        intake.setSpeed(speed);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        intake.setRetractedAngle();
        intake.setSpeed(MetersPerSecond.of(0));
    }

    @Override
    public boolean isFinished() {
        return false; // Runs until interrupted (e.g. button released, or auto ends marker)
    }

    /**
     * Factory for use in PathPlanner named commands.
     * Runs the intake at full speed.
     */
    public static Command forAuto() {
        return new ActivateIntakeCommand(MetersPerSecond.of(MOTOR_SPEED)).withTimeout(AUTO_INTAKE_SECONDS);
    }

    /**
     * Factory to stop the intake in auto.
     * Retracts and stops the motor immediately.
     */
    public static Command forAutoOff() {
        return new Command() {
            private final IntakeSub intake = Subsystems.intake;

            {
                addRequirements(intake);
            }

            @Override
            public void initialize() {
                intake.setRetractedAngle();
                intake.setSpeed(MetersPerSecond.of(0));
            }

            @Override
            public boolean isFinished() {
                return true; // Instant command — runs once and finishes
            }
        };
    }
}