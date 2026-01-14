package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Subsystems;
import frc.robot.constants.DriveConstants;
import frc.robot.utils.ClosedSlewRateLimiter;
import frc.robot.utils.OptimisedSwerveModuleState;
import frc.robot.utils.SwerveModule;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSub extends SubsystemBase {
  private final ClosedSlewRateLimiter xLimiter = new ClosedSlewRateLimiter(
      DriveConstants.MAX_ACCELERATION.in(MetersPerSecondPerSecond),
      DriveConstants.MAX_DECELERATION.in(MetersPerSecondPerSecond));
  private final ClosedSlewRateLimiter yLimiter = new ClosedSlewRateLimiter(
      DriveConstants.MAX_ACCELERATION.in(MetersPerSecondPerSecond),
      DriveConstants.MAX_DECELERATION.in(MetersPerSecondPerSecond));
  private final ClosedSlewRateLimiter rLimiter = new ClosedSlewRateLimiter(
      DriveConstants.MAX_ANGULAR_ACCELERATION.in(RadiansPerSecondPerSecond),
      DriveConstants.MAX_ANGULAR_DECELERATION.in(RadiansPerSecondPerSecond));

  private final SwerveModule frontLeft = new SwerveModule(DriveConstants.SWERVE_MODULE_FL);
  private final SwerveModule frontRight = new SwerveModule(DriveConstants.SWERVE_MODULE_FR);
  private final SwerveModule backLeft = new SwerveModule(DriveConstants.SWERVE_MODULE_BL);
  private final SwerveModule backRight = new SwerveModule(DriveConstants.SWERVE_MODULE_BR);

  private final PIDController holdYawPid = new PIDController(0.8, 0, 0);

  /** Creates a new DriveSubsystem. */
  public DriveSub() {
    setupSmartDash();
  }

  public void driveAtAngle(ChassisSpeeds speeds, boolean fieldRelative, Rotation2d yawAngle) { // TODO
    Rotation2d currentYaw = Subsystems.nav.getHeadingR2D();
    Rotation2d err = currentYaw.minus(yawAngle);
    speeds.omegaRadiansPerSecond = holdYawPid.calculate(err.getDegrees());
    drive(speeds, fieldRelative, true);
  }

  /**
   * drives the robot
   * 
   * @param speeds        speeds to drive at
   * @param fieldRelative true if the provided speeds are field-relative, false if
   *                      they are robot-relative
   */
  public void drive(ChassisSpeeds speeds, boolean fieldRelative, boolean accelerationLimit) {
    if (fieldRelative) { // convert field rel speeds to robot rel
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, new Rotation2d(Subsystems.nav.getDriveHeading()));
    }

    if (accelerationLimit) {
      final var currentSpeeds = Subsystems.nav.getChassisSpeeds();
      speeds.vxMetersPerSecond = xLimiter.calculate(speeds.vxMetersPerSecond, currentSpeeds.vxMetersPerSecond);
      speeds.vyMetersPerSecond = yLimiter.calculate(speeds.vyMetersPerSecond, currentSpeeds.vyMetersPerSecond);
      speeds.omegaRadiansPerSecond = rLimiter.calculate(speeds.omegaRadiansPerSecond,
          currentSpeeds.omegaRadiansPerSecond);
    }

    final var states = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(speeds);
    setModuleStates(states);
  }

  private short logRateCounter = 0;
  @Override
  public void periodic() {    
    if   (logRateCounter < 10) {logRateCounter++; return;} //log every 200ms
    else                       {logRateCounter = 0;}
    log();
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    frontLeft.setDesiredState(new OptimisedSwerveModuleState(0, Rotation2d.fromDegrees(45)));
    frontRight.setDesiredState(new OptimisedSwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    backLeft.setDesiredState(new OptimisedSwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    backRight.setDesiredState(new OptimisedSwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the desired swerve module states.
   *
   * @param desiredStates The desired {@link SwerveModuleState}s.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.MAX_SPEED);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
  }

  /** @return the modules' current {@link SwerveModuleState}s */
  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
        frontLeft.getState(),
        frontRight.getState(),
        backLeft.getState(),
        backRight.getState()
    };
  }

  /** @return the desired {@link SwerveModuleState}s of the modules */
  public SwerveModuleState[] getModuleDesiredStates() {
    return new SwerveModuleState[] {
        frontLeft.getDesiredState(),
        frontRight.getDesiredState(),
        backLeft.getDesiredState(),
        backRight.getDesiredState()
    };
  }

  /** @return the {@link SwerveModulePosition}s of the modules */
  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
        frontLeft.getPosition(),
        frontRight.getPosition(),
        backLeft.getPosition(),
        backRight.getPosition()
    };
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    frontLeft.resetEncoders();
    frontRight.resetEncoders();
    backLeft.resetEncoders();
    backRight.resetEncoders();
  }

  public SequentialCommandGroup testFunction() {
    return new SequentialCommandGroup(
        frontLeft.testFunction(),
        frontRight.testFunction(),
        backLeft.testFunction(),
        backRight.testFunction());
  }

  private void log(){
    frontLeft.log();
    frontRight.log();
    backLeft.log();
    backRight.log();
  }

  @Override
  public void simulationPeriodic() {
    frontLeft.runSim();
    frontRight.runSim();
    backLeft.runSim();
    backRight.runSim();
    
  }

  public void setupSmartDash() {
    SmartDashboard.putData("Swerve Drive Sub", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveDrive");

        builder.addDoubleProperty("Front Left Angle", () -> frontLeft.getTurnAngle().in(Radians), null);
        builder.addDoubleProperty("Front Left Velocity", () -> frontLeft.getDriveVelocity().in(MetersPerSecond), null);

        builder.addDoubleProperty("Front Right Angle", () -> frontRight.getTurnAngle().in(Radians), null);
        builder.addDoubleProperty("Front Right Velocity", () -> frontRight.getDriveVelocity().in(MetersPerSecond),
            null);

        builder.addDoubleProperty("Back Left Angle", () -> backLeft.getTurnAngle().in(Radians), null);
        builder.addDoubleProperty("Back Left Velocity", () -> backLeft.getDriveVelocity().in(MetersPerSecond), null);

        builder.addDoubleProperty("Back Right Angle", () -> backRight.getTurnAngle().in(Radians), null);
        builder.addDoubleProperty("Back Right Velocity", () -> backRight.getDriveVelocity().in(MetersPerSecond), null);

        builder.addDoubleProperty("Robot Angle",
            () -> Subsystems.nav != null ? Subsystems.nav.getHeading().in(Radians) : 0, null);
      }
    });
  }
}
