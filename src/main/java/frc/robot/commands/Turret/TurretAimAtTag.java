package frc.robot.commands.Turret;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;

public class TurretAimAtTag  extends Command{
    final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    
    //TODO
    // while triggered
    // select the nearest apriltag pointed in our direction
    // calc to aim at it
    // do it
    
    //fieldLayout.getTags()

    public TurretAimAtTag(){
        addRequirements(Subsystems.turretAiming, Subsystems.turretFeeder);
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
    }

    @Override
    public void execute() {
        
      
       
    }

    @Override
    public void end(boolean interrupted) {
     
        Subsystems.turretAiming.stop();
        super.end(interrupted);
    }
}
