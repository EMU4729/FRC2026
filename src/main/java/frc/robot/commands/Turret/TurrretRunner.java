package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.OI;
import frc.robot.Subsystems;
import frc.robot.constants.AimingConstants;
import frc.robot.utils.TurretAiming;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;;


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
    public void execute() {
        if (DriverStation.isDisabled()) return;

        // TODO Auto-generated method stub
        FieldArea fieldArea = getFieldArea();
        if(OI.pilot.a().getAsBoolean()){
            CommandScheduler.getInstance().schedule(new TurretAimAtTag());
            SmartDashboard.putBoolean("Turret Inhibit", true);
            return;
        } else {
            SmartDashboard.putBoolean("Turret Inhibit", false);
        }

        if (fieldArea == FieldArea.OurAlliance && OurHubActive()) {     
            CommandScheduler.getInstance().schedule(new TurretShootAtHub());
            SmartDashboard.putString("Shooting Stage", "Shooting At Hub");
        } else if (fieldArea == FieldArea.Neutral || fieldArea == FieldArea.TheirAlliance) {
            CommandScheduler.getInstance().schedule(new TurretPassToHome());
             SmartDashboard.putString("Shooting Stage", "Passing To Home");
        } else {
            //aim at tag
            CommandScheduler.getInstance().schedule(new TurretAimAtTag());
             SmartDashboard.putString("Shooting Stage", "Aiming at Tag");
        }


    }


    private FieldArea getFieldArea(){
        if(DriverStation.getAlliance().isEmpty()) {
            return FieldArea.Blocked;
        }
        //TODO
        // using AimingConstants.RED_Alliance_BOUNDS etc compared to TurretAiming.getTurretPose2d and Subsystems.nav.getPose
        // find and return the area both are in
        // or return blocked
        //DriverStation.getAlliance() == DriverStation.Alliance.Red;
        Pose2d turretPose = TurretAiming.getTurretPose2d();
        Pose2d navPose =  Subsystems.nav.getPose();

        if (inBounds(navPose, AimingConstants.RED_Alliance_BOUNDS) && inBounds(turretPose, AimingConstants.RED_Alliance_BOUNDS) )   {
           if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
                return FieldArea.OurAlliance;
           } else {
                return FieldArea.TheirAlliance;
           }
        }

        if (inBounds(navPose, AimingConstants.BLUE_Alliance_BOUNDS) && inBounds(turretPose, AimingConstants.BLUE_Alliance_BOUNDS))  {
            if(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
                return FieldArea.OurAlliance;
            } else {
                return FieldArea.TheirAlliance;
            }
        } 

        if (inBounds(navPose, AimingConstants.NEUTRAL_BOUNDS) && inBounds(turretPose, AimingConstants.NEUTRAL_BOUNDS)){
            return FieldArea.Neutral;
        }

        return FieldArea.Blocked;
    }

    public boolean OurHubActive() {
        double matchTime = DriverStation.getMatchTime();
        HubOrder WhoActiveFirst = BlueAciveFirst();
        if (HubOrder.TBD == WhoActiveFirst) {
            return true;
        }

        if (matchTime > 130 - AimingConstants.TimerOffset) {
         // Transition shift, hub is active.
            return true;
        } else if (matchTime > 105 - AimingConstants.TimerOffset) {
         // Shift 1
            return WhoActiveFirst == HubOrder.UsFirst;
        } else if (matchTime > 80 - AimingConstants.TimerOffset) {
            // Shift 2
            return WhoActiveFirst == HubOrder.ThemFirst;
        } else if (matchTime > 55 - AimingConstants.TimerOffset) {
        // Shift 3
            return WhoActiveFirst == HubOrder.UsFirst;
        } else if (matchTime > 30 - AimingConstants.TimerOffset) {
            // Shift 4
            return WhoActiveFirst == HubOrder.ThemFirst;
        } else {
        // End game, hub always active.
            return true;
        }
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    private HubOrder BlueAciveFirst(){
        if (DriverStation.getAlliance().isEmpty()){
            return HubOrder.TBD;
        }
        String gameData;
        gameData = DriverStation.getGameSpecificMessage();
        if(gameData.length() > 0) {
            switch (gameData.charAt(0)){
            case 'B' :
                return DriverStation.getAlliance().get() == Alliance.Blue ? HubOrder.UsFirst : HubOrder.ThemFirst;
            case 'R' :
                return DriverStation.getAlliance().get() == Alliance.Red ? HubOrder.UsFirst : HubOrder.ThemFirst;
            default :
                return HubOrder.TBD;
            }
        } else {
            return HubOrder.TBD;
        }
    }

    enum HubOrder {
        UsFirst,
        ThemFirst,
        TBD,
    }

    private boolean inBounds(Pose2d boundsPosition, Translation2d[] boundsList) {
         return (boundsPosition.getX() >= boundsList[0].getX() &&
                 boundsPosition.getX() <= boundsList[1].getX() &&
                 boundsPosition.getY() >= boundsList[0].getY() &&
                 boundsPosition.getY() <= boundsList[1].getY() );
    }

    enum FieldArea {
        OurAlliance,
        TheirAlliance,
        Neutral,
        Blocked
    }
}
