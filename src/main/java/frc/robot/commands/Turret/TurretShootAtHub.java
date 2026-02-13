package frc.robot.commands.Turret;

import java.util.Optional;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;
import frc.robot.constants.AimingConstants;
import frc.robot.constants.AimingConstants.TurretState;
import frc.robot.utils.TurretAiming;

public class TurretShootAtHub  extends Command{
    
    //TODO
    // while triggered
    // pick the red or blue hub
    // calc to shoot there
    // do it

    //AimingConstants.ShootingSamples
    private Translation2d OurHub = AimingConstants.Red_Hub;

    public TurretShootAtHub(){
        
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
        Optional<Alliance> PresentAlliance = DriverStation.getAlliance();
        if (PresentAlliance.isPresent() && PresentAlliance.get() == DriverStation.Alliance.Red) {
            OurHub = AimingConstants.Red_Hub;
        } else {
            OurHub = AimingConstants.Blue_Hub;
        }
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
        super.execute();
        TurretState HubCalc = TurretAiming.calcState(AimingConstants.ShootingSamples, OurHub);
        Subsystems.turretAiming.setHoodTarget(HubCalc.hoodAngle());
        Subsystems.turretAiming.setSlewTarget(HubCalc.turretAngle());
        Subsystems.turretShooter.setSpeed(HubCalc.power());
        Subsystems.turretFeeder.PopFuel();
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
        Subsystems.turretAiming.stop();
        Subsystems.turretShooter.stop();
        Subsystems.turretFeeder.stop();
    }   
    
}
