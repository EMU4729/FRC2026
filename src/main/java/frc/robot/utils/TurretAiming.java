package frc.robot.utils;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import java.io.PrintStream;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Subsystems;
import frc.robot.constants.AimingConstants;
import frc.robot.constants.AimingConstants.DistanceSample;
import frc.robot.constants.AimingConstants.TurretState;

public class TurretAiming {
     private static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    // Offset from Robot Center to Turret Pivot (X, Y, Z)
    public static final Transform3d turretOffset = new Transform3d(new Translation3d(0.3, 0.3, 0.4), new Rotation3d()); //TODO VERIFIY THIS



    public static TurretState calcState(List<DistanceSample> sampleList, Translation2d target){

       
        
        //1. Define Euclidean Geometry of the Target
        double target_transform = target.getNorm();
        double target_angle = target.getAngle().getDegrees();

        DistanceSample LowerBounds = null;
        DistanceSample UpperBounds = null;
        
        for (DistanceSample sample :sampleList)
        {
           
            if (sample.distance().in(Meters)<= target_transform){
                     if (LowerBounds == null || sample.distance().in(Meters) > LowerBounds.distance().in(Meters)){
                        LowerBounds = sample;
                     }
            }
            if (sample.distance().in(Meters)>= target_transform){
                     if (UpperBounds == null || sample.distance().in(Meters) > UpperBounds.distance().in(Meters)){
                        UpperBounds = sample;
                     }
            }
        }

        Angle AngleToTarget = Degrees.of(target_angle);
        if (LowerBounds ==null) return UpperBounds.toTurretState(AngleToTarget);
        if (UpperBounds == null) return LowerBounds.toTurretState(AngleToTarget);
        if (LowerBounds == UpperBounds) return LowerBounds.toTurretState(AngleToTarget);
      
    double t = (target_transform - LowerBounds.distance().in(Meters))/ (UpperBounds.distance().in(Meters) - LowerBounds.distance().in(Meters));
    double interpolatedPower = LowerBounds.power().in(MetersPerSecond) + t * (UpperBounds.power().in(MetersPerSecond) - LowerBounds.power().in(MetersPerSecond));
    double interpolatedHood = LowerBounds.hoodAngle().in(Degrees) + t * (UpperBounds.hoodAngle().in(Degrees) - LowerBounds.hoodAngle().in(Degrees));
    
    
        return new TurretState(AngleToTarget, MetersPerSecond.of(interpolatedPower), Degrees.of(interpolatedHood));
    }

    private static DistanceSample interpolateDistance(List<DistanceSample> sampleList, Distance distance){
        //TODO 
    double targetMeters = distance.in(Meters);

    if (sampleList.isEmpty()) return new DistanceSample(null, null, distance);
    if (sampleList.size() == 1) return sampleList.get(0);

    DistanceSample lower = sampleList.get(0);
    DistanceSample upper = sampleList.get((sampleList.size() -1));


    if (targetMeters <= lower.distance().in(Meters)) return lower;
    if (targetMeters >= upper.distance().in(Meters)) return upper;

    for (int i =0; i< sampleList.size() -1; i++){
         if (targetMeters>= sampleList.get(i).distance().in(Meters) &&
             targetMeters <= sampleList.get(i+1).distance().in(Meters)){
                    lower = sampleList.get(i);
                    upper = sampleList.get(i+1);
                    break;
            }
        }   


double lowDist = lower.distance().in(Meters);
double highDist = upper.distance().in(Meters);
double t = (targetMeters - lowDist) / (highDist - lowDist);


double interpolatedPower = MathUtil.interpolate(lower.power().in(MetersPerSecond), upper.power().in(MetersPerSecond), t);

double interpolatedHood = MathUtil.interpolate(lower.hoodAngle().in(Degrees), upper.hoodAngle().in(Degrees), t);

        return new DistanceSample(MetersPerSecond.of(interpolatedPower), Degrees.of(interpolatedHood), distance); 
        
    }

    private static Distance getDistance(Transform2d transform){
        return Meters.of(transform.getTranslation().getNorm());
    }

    public static Pose2d getTurretPose2d(){
        return Subsystems.nav.getPose().transformBy(AimingConstants.robotToTurret);
    }

    /** Returns either robot relative or field relative  */
 public static Optional<Rotation2d> getTargetAprilTagAngle(Pose2d robotPose, boolean fieldRelative) {
        Optional<AprilTag> targetTag = getBestTarget(robotPose);

        if (targetTag.isEmpty()) {
            return Optional.empty();
        }

        Translation2d tagTranslation = targetTag.get().pose.getTranslation().toTranslation2d();
        
        // 1. Calculate Field-Relative Angle (Angle from robot to tag on the map)
        Rotation2d fieldAngle = tagTranslation.minus(robotPose.getTranslation()).getAngle();

        if (fieldRelative) {
            return Optional.of(fieldAngle);
        } else {
            // 2. Calculate Robot-Relative Angle (Field Angle - Robot Heading)
            return Optional.of(fieldAngle.minus(robotPose.getRotation()));
        }
    }

  
 public static Alliance getAlliance(){
        return DriverStation.getAlliance().orElse(Alliance.Blue);
}

/** Selects the best tag based on distance and a 45-degree facing filter */
    public static Optional<AprilTag> getBestTarget(Pose2d robotPose) {
        AprilTag bestTag = null;
        double closestDistance = Double.MAX_VALUE;
        double maxAngleTolerance = Math.toRadians(45); 

        for (AprilTag tag : fieldLayout.getTags()) {
            if (getAlliance() == Alliance.Red && tag.ID > 16) continue;
         if (getAlliance() == Alliance.Blue && tag.ID <= 16) continue;
            Translation2d tagToRobotTrans = robotPose.getTranslation().minus(tag.pose.getTranslation().toTranslation2d());
            
            // Calculate if the robot is actually in front of the tag
            Rotation2d angleToRobot = new Rotation2d(tagToRobotTrans.getX(), tagToRobotTrans.getY());
            Rotation2d tagFacing = tag.pose.getRotation().toRotation2d();
            double angleDiff = Math.abs(MathUtil.angleModulus(tagFacing.minus(angleToRobot).getRadians()));
           //angleDiff = Math.IEEEremainder(angleDiff, 2.0 * Math.PI);

            if (angleDiff > maxAngleTolerance) continue; 

            double distance = tagToRobotTrans.getNorm();
            if (distance < closestDistance) {
                closestDistance = distance;
                bestTag = tag;
            }
        }
        return Optional.ofNullable(bestTag);
    }
   
    }
    
