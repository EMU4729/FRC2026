package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;
import frc.robot.subsystems.IntakeSub;

public class ActivateIntakeCommand extends Command{
    public static final double MOTOR_SPEED = 1.0;
    IntakeSub intake = Subsystems.intake;
    public ActivateIntakeCommand(){
        addRequirements(intake);
    }
    @Override
    public void initialize() {
        intake.setExtendAngle();
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted){
        intake.setRetractedAngle();
    }
}