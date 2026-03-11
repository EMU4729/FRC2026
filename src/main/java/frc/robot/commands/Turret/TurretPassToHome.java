package frc.robot.commands.Turret;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import static edu.wpi.first.units.Units.Radians;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;
import frc.robot.constants.AimingConstants;
import frc.robot.constants.AimingConstants.TurretState;
import frc.robot.utils.TurretAiming;


public class TurretPassToHome  extends Command{
    
    private Translation2d[] targets;

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
        Translation2d targetPos = getPassingTarget(Subsystems.nav.getPose().getTranslation());
        Subsystems.nav.drawFieldObject("TurretTarget", new Pose2d(targetPos, new Rotation2d()), false);

    TurretState targetState = TurretAiming.calcState(AimingConstants.PassingSamples, targetPos);

    // Rotate robot to face the passing target
    Subsystems.drive.driveAtAngle(new ChassisSpeeds(0, 0, 0), true,
        Rotation2d.fromRadians(targetState.turretAngle().in(Radians)));

    Subsystems.turretAiming.setHoodTarget(targetState.hoodAngle());
    Subsystems.turretShooter.setSpeed(targetState.power());
        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        Subsystems.turretFeeder.stop();
        super.end(interrupted);
    }

    private Translation2d getPassingTarget(Translation2d robotPos) {
        Translation2d bestTarget = targets[0]; // Start with the first one as a baseline
        double dx = robotPos.getX() - bestTarget.getX();
        double dy = robotPos.getY() - bestTarget.getY();
        double closestDistanceSq = dx * dx + dy * dy;

        for (Translation2d target : targets) {
            double targetDx = robotPos.getX() - target.getX();
            double targetDy = robotPos.getY() - target.getY();
            double distanceSq = targetDx * targetDx + targetDy * targetDy;
            if (distanceSq < closestDistanceSq) {
                closestDistanceSq = distanceSq;
                bestTarget = target;
            }
        }

        return bestTarget;
    }
}
