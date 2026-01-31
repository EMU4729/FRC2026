package frc.robot.utils;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;

import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Subsystems;
import frc.robot.constants.AimingConstants;
import frc.robot.constants.AimingConstants.DistanceSample;
import frc.robot.constants.AimingConstants.TurretState;

public class TurretAiming {
    



    public static TurretState calcState(List<DistanceSample> sampleList, Pose2d target){
        //TODO
        // Find the distance and angle to the target from the turret
        // interpolate for power and hood angle
        // create and return a Turret state of those values
        return new TurretState(Degrees.of(0), DegreesPerSecond.of(0), Degrees.of(0));
    }

    private static DistanceSample interpolateDistance(List<DistanceSample> sampleList, Distance distance){
        //TODO 
        // search the list for the next largest/smallest distance
        // use interpolation 'MathUtil.interpolate(0, 0, 0);' to find the best power / angle to shoot
        // t = (x - smaller) / (larger - smaller)
        // return
        return new DistanceSample(DegreesPerSecond.of(0), Degrees.of(0), Meters.of(0)); 
        
    }

    private static Distance getDistance(Transform2d transform){
        return Meters.of(transform.getTranslation().getNorm());
    }

    public static Pose2d getTurretPose2d(){
        return Subsystems.nav.getPose().transformBy(AimingConstants.robotToTurret);
    }
}
