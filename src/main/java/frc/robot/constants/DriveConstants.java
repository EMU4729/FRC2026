
package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.utils.rangemath.AxesFit;
import frc.robot.utils.rangemath.DriveBaseFit;

public class DriveConstants {
  /** Distance between centers of left and right wheels on robot in meters */
  public static final Distance TRACK_WIDTH = Meters.of(0.52);

  /** Distance between front and back wheel on robot in meters */
  public static final Distance WHEEL_BASE = Meters.of(0.52);

  /** Drivebase radius in m (distance from center of robot to farthest module) */
  public static final Distance DRIVEBASE_RADIUS = Meters
      .of(Math.hypot(WHEEL_BASE.in(Meters) / 2, TRACK_WIDTH.in(Meters) / 2));

  public static final Translation2d[] FIELD_BOUNDS = { new Translation2d(0, 0), new Translation2d(16.54, 8.07) };


  // Driving Parameters - Note that these are not the maximum capable speeds of
  // the robot, rather the allowed maximum speeds
  /**
   * Max speed of robot in meters per second
   * 
   * Right now, this is just set to a bit below the maxswerve module's free speed.
   * It should probably be changed.
   */
  public static final LinearVelocity MAX_SPEED = MetersPerSecond.of(5);
  /** Max acceleration of robot in meters per second squared */
  public static final LinearAcceleration MAX_ACCELERATION = MetersPerSecondPerSecond.of(0.3); // TODO check this
  /** Max (negative) deceleration of the robot */
  public static final LinearAcceleration MAX_DECELERATION = MAX_ACCELERATION.times(-2);
  /**
   * Max angular speed of robot in radians per second
   * 
   * This is derived from the MAX_SPEED using the angular velocity formula v = ωr
   */
  public static final AngularVelocity MAX_ANGULAR_SPEED = RadiansPerSecond
      .of(MAX_SPEED.in(MetersPerSecond) / DRIVEBASE_RADIUS.in(Meters));
  /** Max angular acceleration of robot in radians per second squared */
  public static final AngularAcceleration MAX_ANGULAR_ACCELERATION = RadiansPerSecondPerSecond
      .of(MAX_ANGULAR_SPEED.in(RadiansPerSecond) / 60 * 15);
  public static final AngularAcceleration MAX_ANGULAR_DECELERATION = RadiansPerSecondPerSecond
      .of(-(MAX_ANGULAR_SPEED.in(RadiansPerSecond) / 60 * 30));

  public static final PathConstraints PATH_CONSTRAINTS = new PathConstraints(
      MAX_SPEED, MAX_ACCELERATION,
      MAX_ANGULAR_SPEED, MAX_ANGULAR_ACCELERATION);

  /**
   * Gear ratio of the MAX Swerve Module driving motor (gear ratio upgrade kit
   * extra high speed 1)
   * 
   * note to self: read deeper and double check unintuitive science next time,
   * Joel
   * 
   * @see https://www.revrobotics.com/rev-21-3005/ for gear ratios
   */
  public static final double DRIVE_GEAR_RATIO = 4.50;

  // Chassis configuration

  // Module Constants

  // Calculations required for driving motor conversion factors and feed forward
  /** Wheel diameter in meters */
  public static final Distance WHEEL_DIAMETER = Inches.of(3);
  /** Wheel circumference in meters */
  public static final Distance WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER.times(Math.PI);

  /** Turning encoder position factor */
  public static final double TURNING_ENCODER_POSITION_FACTOR = (2 * Math.PI); // radians
  /** Turning encoder velocity factor */
  public static final double TURNING_ENCODER_VELOCITY_FACTOR = TURNING_ENCODER_POSITION_FACTOR / 60.0; // rad/s
  public static final double TURNING_ENCODER_POSITION_PID_MIN_INPUT = 0; // radians
  public static final double TURNING_ENCODER_POSITION_PID_MAX_INPUT = TURNING_ENCODER_POSITION_FACTOR; // radians

  // TODO tune PID
  public static final double DRIVE_P = 1.6;// 0.7;
  public static final double DRIVE_I = 5;// 8;
  public static final double DRIVE_D = 0;
  public static final double DRIVING_FF = 0;

  public static final double TURNING_P = 1.3;
  public static final double TURNING_I = 0.05;
  public static final double TURNING_D = 0.15;
  public static final double TURNING_FF = 0;
  public static final Angle TURNING_I_ZONE = Degrees.of(15);

  // Auto Constants
  // TODO: Tune these
  /** Auto translation PID constants */
  public static final PIDConstants AUTO_TRANSLATION_PID = new PIDConstants(10, 1, 0);
  /** Auto rotation PID constants */
  public static final PIDConstants AUTO_ROTATION_PID = new PIDConstants(5, 1, 0);

  /*
   * public static DriveBaseFit PILOT_SETTINGS = DriveBaseFit(
   * 0, 1, 4, 0.1, true,
   * 0, 1, 4, 0.1, false,
   * 0, 1, 3, 0.1, false,
   * 0.85, 0.8);
   */
  public static DriveBaseFit PILOT_SETTINGS = new DriveBaseFit(
      new AxesFit().withOutputMinMax(0, 0.7).withPow(4).withDeadBand(0.1)
          .withLimiter(0.15).withBooster(1),
      new AxesFit().withPow(3).withDeadBand(0.1).withLimiter(0.15));

  /*
   * public static DriveBaseFit PILOT_DEMO_SETTINGS = DriveBaseFit.InitSwerveBot(
   * 0, 0.2, 2, 0.1, true,
   * 0, 0.2, 2, 0.1, false,
   * 0, 0.2, 2, 0.1, false,
   * 0.6, 1);
   */
  public static DriveBaseFit PILOT_DEMO_SETTINGS = new DriveBaseFit(
      new AxesFit().withOutputMinMax(0, 0.15).withPow(2).withDeadBand(0.1).withBooster(0.35),
      new AxesFit().withOutputMinMax(0, 0.2).withPow(2).withDeadBand(0.1));

  public static final SwerveModuleDetails SWERVE_MODULE_FL = new SwerveModuleDetails(
      "Swerve_FL",
      1, // Drive motor CAN ID
      1, // Steer motor CAN ID
      Rotation2d.kCW_90deg, // offset relative to FL
      false,
      new Translation2d(WHEEL_BASE.div(2), TRACK_WIDTH.div(2)) // location rel to centre
  );
  public static final SwerveModuleDetails SWERVE_MODULE_FR = new SwerveModuleDetails(
      "Swerve_FR",
      2, // Drive motor CAN ID
      2, // Steer motor CAN ID
      Rotation2d.kZero, // offset relative to FL
      true,

      new Translation2d(WHEEL_BASE.div(2), TRACK_WIDTH.div(-2)) // location rel to centre
  );

  public static final SwerveModuleDetails SWERVE_MODULE_BL = new SwerveModuleDetails(
      "Swerve_BL",
      3, // Drive motor CAN ID
      3, // Steer motor CAN ID
      Rotation2d.k180deg, // Offset rel to FL module
      true,
      new Translation2d(WHEEL_BASE.div(-2), TRACK_WIDTH.div(2)) // location rel to centre
  );
  public static final SwerveModuleDetails SWERVE_MODULE_BR = new SwerveModuleDetails(
      "Swerve_BR",
      4, // Drive motor CAN ID
      4, // Steer motor CAN ID
      Rotation2d.kCCW_90deg, // Offset rel to FL module
      true,
      new Translation2d(WHEEL_BASE.div(-2), TRACK_WIDTH.div(-2)) // location rel to centre
  );

  /** Swerve Kinematics */
  public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
      SWERVE_MODULE_FL.location(),
      SWERVE_MODULE_FR.location(),
      SWERVE_MODULE_BL.location(),
      SWERVE_MODULE_BR.location());

  public static record SwerveModuleDetails(
      String name,
      /** CAN ID for the module's Driving Motor */
      int driveCANID,
      /** CAN ID for the module's Steering Motor */
      int steerCANID,
      /**
       * Angular offset of the module around the robot's center, relative to FL
       * module.
       */
      Rotation2d angularOffset,
      /** Whether the drive motor is inverted or not */
      boolean invertDrive,
      /**
       * Location of module relative to robot center. Mainly for sim purposes.
       */
      Translation2d location) {
  }
}
