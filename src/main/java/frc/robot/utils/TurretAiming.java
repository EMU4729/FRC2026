package frc.robot.utils;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;

import java.io.PrintStream;
import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Subsystems;
import frc.robot.constants.AimingConstants;
import frc.robot.constants.AimingConstants.DistanceSample;
import frc.robot.constants.AimingConstants.TurretState;

public class TurretAiming {
    



    public static TurretState calcState(List<DistanceSample> sampleList, Pose2d target){
        //TODO
        //1. Define Euclidean Geometry of the Target
        double target_transform = target.getTranslation().getNorm();
        double target_angle = target.getRotation().getDegrees();

        DistanceSample LowerBounds = null;
        DistanceSample UpperBounds = null;
        
        for (DistanceSample sample :sampleList){
            if (sample.distance().in(Meters)<= target_transform){
                     if (LowerBounds == null || sample.distance().in(Meters) > LowerBounds.distance().in(Meters)){
                        LowerBounds = sample;
                     }
            }
            if (sample.distance().in(Meters)<= target_transform){
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
    double interpolatedPower = LowerBounds.power().in(DegreesPerSecond) + t * (UpperBounds.power().in(DegreesPerSecond) - LowerBounds.power().in(DegreesPerSecond));
    double interpolatedHood = LowerBounds.hoodAngle().in(Degrees) + t * (UpperBounds.hoodAngle().in(Degrees) - LowerBounds.hoodAngle().in(Degrees));
    
    
        return new TurretState(AngleToTarget, DegreesPerSecond.of(interpolatedPower), Degrees.of(interpolatedHood));
    }

    private static DistanceSample interpolateDistance(List<DistanceSample> sampleList, Distance distance){
        //TODO 
double targetMeters = distance.in(Meters);

if (sampleList.isEmpty()) return new DistanceSample(null, null, distance);
if (sampleList.size() == 1) return sampleList.get(0);

DistanceSample lower = sampleList.get(0);
DistanceSample upper = sampleList.get((sampleList.size() -1));


if (targetMeters <= lower.distance().in(Meters)) return lower;
if (targetMeters <= upper.distance().in(Meters)) return upper;

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


double interpolatedPower = MathUtil.interpolate(lower.power().in(DegreesPerSecond), upper.power().in(DegreesPerSecond), t);

double interpolatedHood = MathUtil.interpolate(lower.hoodAngle().in(Degrees), upper.hoodAngle().in(Degrees), distance.in(Meters));

        // search the list for the next largest/smallest distance
        // use interpolation 'MathUtil.interpolate(0, 0, 0);' to find the best power / angle to shoot
        // t = (x - smaller) / (larger - smaller)
        // return
        return new DistanceSample(DegreesPerSecond.of(interpolatedPower), Degrees.of(interpolatedHood), distance); 
        
    }

    private static Distance getDistance(Transform2d transform){
        return Meters.of(transform.getTranslation().getNorm());
    }

    public static Pose2d getTurretPose2d(){
        return Subsystems.nav.getPose().transformBy(AimingConstants.robotToTurret);
    }

  
}
