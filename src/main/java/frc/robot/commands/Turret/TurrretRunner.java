package frc.robot.commands.Turret;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Subsystems;

public class TurrretRunner extends Command{
    //TODO
    // always running
    // if we are in our alliance zone and the hub is active : shoot at it
    // if we are in the neutral or other alliance zone : pass
    // if the driver is holding the lockout button (say 'a') : aim at the best tag
    // if we are not in any zone aim at the best tag

    public TurrretRunner(){

    }

    @Override
    public void initialize() {
        //CommandScheduler.getInstance().schedule(Command...)
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
    

    private FieldArea getFieldArea(){
        //TODO
        // using AimingConstants.RED_Alliance_BOUNDS etc compared to TurretAiming.getTurretPose2d and Subsystems.nav.getPose
        // find and return the area both are in
        // or return blocked
        //DriverStation.getAlliance() == DriverStation.Alliance.Red;
        return FieldArea.Blocked;
    }

    enum FieldArea {
        OurAlliance,
        TheirAlliance,
        Neutral,
        Blocked
    }
}
