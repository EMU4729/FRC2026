package frc.robot.commands.Turret;

import java.util.List;
import java.util.Optional;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;
import frc.robot.constants.AimingConstants;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

public class TurretPassToHome  extends Command{
    
    //TODO
    // while triggered
    // select the left or right target
    // calc to shoot there
    // do it

    //AimingConstants.PassingSamples

    public TurretPassToHome(){
        addRequirements(Subsystems.turretFeeder);
    }

 

    @Override
    public void execute() {
      Pose2d robotPose = Subsystems.drive.getPose();
    Translation2d targetPos = getPassingTarget(robotPose);
    
    double distanceToTarget = robotPose.getTranslation().getDistance(targetPos);

    // 1. Calculate Angles
    Rotation2d fieldAngle = targetPos.minus(robotPose.getTranslation()).getAngle();
    Rotation2d robotRelativeAngle = fieldAngle.minus(robotPose.getRotation());

    // 2. Get interpolated AngularVelocity (Power) from your DistanceSample list
    AngularVelocity targetPower = getInterpolatedPower(distanceToTarget);

    // 3. Command Subsystem
    Subsystems.turretFeeder.setTargetAngle(robotRelativeAngle);
    
    
    Subsystems.turretFeeder.setSpeedFromAngular(targetPower);
        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        Subsystems.turretFeeder.stop();
        super.end(interrupted);
    }
private AngularVelocity getInterpolatedPower(double distance) {
    List<AimingConstants.DistanceSample> samples = AimingConstants.PassingSamples;
    
    if (samples.isEmpty()) return DegreesPerSecond.of(0);
    if (samples.size() == 1) return samples.get(0).power();

    // Find bounding samples
    AimingConstants.DistanceSample low = samples.get(0);
    AimingConstants.DistanceSample high = samples.get(samples.size() - 1);

    for (int i = 0; i < samples.size() - 1; i++) {
        if (distance >= samples.get(i).distance().in(Meters) && 
            distance <= samples.get(i + 1).distance().in(Meters)) {
            low = samples.get(i);
            high = samples.get(i + 1);
            break;
        }
    }

    // Interpolation math for AngularVelocity
    double lowDist = low.distance().in(Meters);
    double highDist = high.distance().in(Meters);
    double range = highDist - lowDist;
    double fraction = (range == 0) ? 0 : (distance - lowDist) / range;

    double lerpedPower = low.power().in(DegreesPerSecond) + 
        fraction * (high.power().in(DegreesPerSecond) - low.power().in(DegreesPerSecond));

    return DegreesPerSecond.of(lerpedPower);
}
    private Translation2d getPassingTarget(Pose2d robotPose) {
       // 1. Determine which array of targets to use based on Alliance
    Translation2d[] targets;
    
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
