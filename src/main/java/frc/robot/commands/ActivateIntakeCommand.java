package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.IntakeSub;

public class ActivateIntakeCommand extends Command{
    private LinearVelocity speed;
    public static final double MOTOR_SPEED = 1.0;
    IntakeSub intake = Subsystems.intake;
    public ActivateIntakeCommand(LinearVelocity speed){
        this.speed = speed;
        addRequirements(intake);
    }
    @Override
    public void initialize() {
        intake.setSpeed(MetersPerSecond.of(0));
    }

    @Override
    public void execute() {
        intake.setSpeed(speed);
    }

    @Override
    public void end(boolean interrupted){
        intake.setSpeed(MetersPerSecond.of(0));
    }
}