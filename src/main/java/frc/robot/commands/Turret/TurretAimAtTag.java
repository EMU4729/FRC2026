package frc.robot.commands.Turret;

import static edu.wpi.first.units.Units.Radians;

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
        addRequirements(Subsystems.turretAiming);
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
    }

    @Override
    public void execute() {
        
      
       
    }

    private Optional<AprilTag> getBestTarget(Pose2d robotPose) {
    AprilTag bestTag = null;
    double closestDistance = Double.MAX_VALUE;
    // 45 degrees in radians
    double maxAngleTolerance = Math.toRadians(45); 

    for (AprilTag tag : fieldLayout.getTags()) {
        // 1. Get vector from Tag to Robot
        Translation2d tagToRobotTrans = robotPose.getTranslation().minus(tag.pose.getTranslation().toTranslation2d());
        
        // 2. Check the angle of incidence
        // This is the angle from the Tag's face to the Robot
        Rotation2d angleToRobot = new Rotation2d(tagToRobotTrans.getX(), tagToRobotTrans.getY());
        Rotation2d tagFacing = tag.pose.getRotation().toRotation2d();
        
        // Use WPILib's minus() to get the shortest distance between rotations
        double angleDiff = Math.abs(tagFacing.minus(angleToRobot).getRadians());

        // 3. Filter if we are viewing the tag from too sharp an angle
        if (angleDiff > maxAngleTolerance) {
            continue; 
        }

        // 4. Standard proximity check
        double distance = tagToRobotTrans.getNorm();
        if (distance < closestDistance) {
            closestDistance = distance;
            bestTag = tag;
        }
    }
    return Optional.ofNullable(bestTag);
    }

    @Override
    public void end(boolean interrupted) {
     
        Subsystems.turretAiming.stop();
        super.end(interrupted);
    }
}
