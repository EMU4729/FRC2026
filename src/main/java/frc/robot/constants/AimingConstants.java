package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

public class AimingConstants {
  public static final Transform2d robotToTurret = new Transform2d(0,0, new Rotation2d(0));


  public static final Translation2d[] RED_Alliance_BOUNDS = { new Translation2d(0, 0), new Translation2d(3.75, 8.07) };
  public static final Translation2d[] BLUE_Alliance_BOUNDS = { new Translation2d(12.79, 0), new Translation2d(16.54, 8.07) };
  public static final Translation2d[] NEUTRAL_BOUNDS = { new Translation2d(5.5, 0), new Translation2d(11.04, 8.07) };

  public static final Translation2d[] Blue_Pass_To_Targets = {new Translation2d(2.5, 1.25), new Translation2d(2.5, 6.75)};
  public static final Translation2d[] Red_Pass_To_Targets = {new Translation2d(14.04, 6.75), new Translation2d(14.04, 1.25)};

  public static final Translation2d Blue_Hub = new Translation2d(4.64, 4.04);
  public static final Translation2d Red_Hub = new Translation2d(9.9, 4.04);
    

  public static final List<DistanceSample> ShootingSamples = Arrays.asList(
      new DistanceSample(DegreesPerSecond.of(0), Degrees.of(0), Meters.of(0)),
      new DistanceSample(DegreesPerSecond.of(0), Degrees.of(0), Meters.of(100))
    );
  public static final List<DistanceSample> PassingSamples = Arrays.asList(
      new DistanceSample(DegreesPerSecond.of(0), Degrees.of(0), Meters.of(0)),
      new DistanceSample(DegreesPerSecond.of(100), Degrees.of(0), Meters.of(100))
    );
  public static record DistanceSample(
    AngularVelocity power,
    Angle hoodAngle,

    Distance distance){

      public TurretState toTurretState(Angle turretAngle){
        return new TurretState(turretAngle, this.power(), this.hoodAngle);
      }
      
  }

  public static record TurretState(
    Angle turretAngle,
    AngularVelocity power,
    Angle hoodAngle){
      
  }

  public static final double TimerOffset = 2;
}
