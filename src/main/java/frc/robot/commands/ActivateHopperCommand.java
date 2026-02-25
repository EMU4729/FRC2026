package frc.robot.commands;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;
import frc.robot.subsystems.HopperSub;
import frc.robot.OI;
import frc.robot.constants.HopperConstants;

public class ActivateHopperCommand extends Command {
    private final LinearVelocity speed;
    private final HopperSub hopper = Subsystems.hopper;

    public ActivateHopperCommand(LinearVelocity speed){
        this.speed = speed;
        addRequirements(hopper);
    }

    @Override
    public void initialize(){
        hopper.setActivated(true);
        hopper.setSpeed(speed);
    }

    @Override
    public void execute(){
        // keep speed set in case anything else changed it
        hopper.setSpeed(speed);
        if (OI.pilot.b().getAsBoolean()){
            hopper.setSpeed(HopperConstants.HOPPER_DEFAULT_SPEED);
        }
    }

    @Override
    public void end(boolean interrupted){
        hopper.setActivated(false);
        hopper.stop();
    }
}
