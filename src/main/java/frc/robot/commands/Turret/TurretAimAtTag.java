package frc.robot.commands.Turret;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
        
        addRequirements(Subsystems.turretFeeder);
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
    }

    @Override
    public void execute() {
        
        Pose2d robotPose = Subsystems.drive.getPose();
        
        // 2. Find the "best" target tag
        Optional<AprilTag> targetTag = getBestTarget(robotPose);

        if (targetTag.isPresent()) {
            Translation2d tagTranslation = targetTag.get().pose.getTranslation().toTranslation2d();
            
            // 1. Calculate Field-Relative Angle (where the tag is on the map)
            Rotation2d fieldAngle = tagTranslation.minus(robotPose.getTranslation()).getAngle();

            // 2. Calculate Robot-Relative Angle (where the turret needs to point)
            // This ensures the turret stays locked even while the drive base spins.
            Rotation2d robotRelativeAngle = fieldAngle.minus(robotPose.getRotation());

            // 3. Command the turret
            Subsystems.turretFeeder.setTargetAngle(robotRelativeAngle);
        }
        super.execute();
    }

    private Optional<AprilTag> getBestTarget(Pose2d robotPose) {
        AprilTag bestTag = null;
        double closestDistance = Double.MAX_VALUE;

        for (AprilTag tag : fieldLayout.getTags()) {
            double distance = robotPose.getTranslation().getDistance(tag.pose.getTranslation().toTranslation2d());
            
            if (distance < closestDistance) {
                closestDistance = distance;
                bestTag = tag;
            }
        }
        return Optional.ofNullable(bestTag);
    }

    @Override
    public void end(boolean interrupted) {
     
        Subsystems.turretFeeder.stop();
        super.end(interrupted);
    }
}
