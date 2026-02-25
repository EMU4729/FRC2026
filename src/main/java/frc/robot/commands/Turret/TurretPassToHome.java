package frc.robot.commands.Turret;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;
import frc.robot.constants.AimingConstants;
import frc.robot.constants.AimingConstants.TurretState;
import frc.robot.utils.TurretAiming;


public class TurretPassToHome  extends Command{
    
    private Translation2d[] targets;
    
    //TODO
    // while triggered
    // select the left or right target
    // calc to shoot there
    // do it

    //AimingConstants.PassingSamples

    public TurretPassToHome(){
        addRequirements(Subsystems.turretAiming, Subsystems.turretFeeder, Subsystems.turretShooter);
    }

    @Override
    public void initialize() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            targets = AimingConstants.Red_Pass_To_Targets;
        } else {
            // Default to Blue if no alliance is found or if it is Blue
            targets = AimingConstants.Blue_Pass_To_Targets;
        }
    }

    @Override
    public void execute() {
        Pose2d robotPose = Subsystems.nav.getPose();
        Translation2d targetPos = getPassingTarget(robotPose);
        Subsystems.nav.drawFieldObject("TurretTarget", new Pose2d(targetPos, new Rotation2d()), false);

        TurretState targetState = TurretAiming.calcState(AimingConstants.PassingSamples, targetPos);

        Subsystems.turretAiming.setSlewTarget(targetState.turretAngle());
        Subsystems.turretAiming.setHoodTarget(targetState.hoodAngle());
        Subsystems.turretShooter.setSpeed(targetState.power());
        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        Subsystems.turretFeeder.stop();
        super.end(interrupted);
    }

    private Translation2d getPassingTarget(Pose2d robotPose) {
        // 1. Determine which array of targets to use based on Alliance
        
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            targets = AimingConstants.Red_Pass_To_Targets;
        } else {
            // Default to Blue if no alliance is found or if it is Blue
            targets = AimingConstants.Blue_Pass_To_Targets;
        }

        // 2. Find the closest target in the selected array
        Translation2d bestTarget = targets[0]; // Start with the first one as a baseline
        double closestDistance = robotPose.getTranslation().getDistance(bestTarget);

        for (Translation2d target : targets) {
            double distance = robotPose.getTranslation().getDistance(target);
            if (distance < closestDistance) {
                closestDistance = distance;
                bestTarget = target;
            }
        }

        return bestTarget;
    }
}
