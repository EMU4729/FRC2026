package frc.robot.commands.Turret;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class TurretPassToHome  extends Command{
    
    //TODO
    // while triggered
    // select the left or right target
    // calc to shoot there
    // do it

    //AimingConstants.PassingSamples

    public TurretPassToHome(){

    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
    }

    private Translation2d getPassingTarget(){
        //TODO
        // use AimingConstants.Blue_Pass_To_Targets and red targets
        // if we are on the blue on the left side of the field return the blue left translation etc

        return new Translation2d();
    }
}
