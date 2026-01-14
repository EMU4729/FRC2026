package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Radians;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
//import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.simulation.ADIS16470_IMUSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Subsystems;
import frc.robot.utils.photon.PhotonBridge;
import frc.robot.utils.photon.PhotonCameraPoseEstimator;
import frc.robot.constants.DriveConstants;
import frc.robot.utils.pathplanner.AutoBuilderFix;

public class NavigationSub extends SubsystemBase {
  private final static ADIS16470_IMU imu = new ADIS16470_IMU(IMUAxis.kX, IMUAxis.kY, IMUAxis.kZ);
  private final Field2d field = new Field2d();
  private final ADIS16470_IMUSim imuSim = new ADIS16470_IMUSim(imu);
  private Pose2d poseSim = new Pose2d();
  public final PhotonBridge photon = new PhotonBridge();
  private final SwerveDrivePoseEstimator poseEstimator;
  private Angle IMURawOffset = Radians.of(0);

  private DataLog datalog = DataLogManager.getLog();
  private DoubleLogEntry LogPoseX = new DoubleLogEntry(datalog, "Pose/X");
  private DoubleLogEntry LogPoseY = new DoubleLogEntry(datalog, "Pose/Y");
  private DoubleLogEntry LogPoseYaw = new DoubleLogEntry(datalog, "Pose/Yaw");
  private DoubleLogEntry LogPoseDriveYaw = new DoubleLogEntry(datalog, "Pose/DriveYaw");


  public NavigationSub() {
    zeroIMUHeading();
    initPathPlanner();

    poseEstimator = new SwerveDrivePoseEstimator(
        DriveConstants.DRIVE_KINEMATICS,
        Rotation2d.fromDegrees(imu.getAngle()),
        Subsystems.drive.getModulePositions(),
        new Pose2d());

    resetOdometry(null);

    SmartDashboard.putData("Field", field);
  }

  /**
   * Initializes PathPlanner.
   * 
   * Should be called once upon robot start.
   */
  private void initPathPlanner() {
    try {
      final var config = RobotConfig.fromGUISettings();
      AutoBuilderFix.configure(
          true,
          this::getPose,
          this::resetOdometry,
          this::getChassisSpeeds,
          (speeds, feedforwards) -> {
            Subsystems.drive.drive(speeds, false, false);
          },
          new PPHolonomicDriveController(
              DriveConstants.AUTO_TRANSLATION_PID,
              DriveConstants.AUTO_ROTATION_PID),
          config,
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red
            // alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;
          },
          this, Subsystems.drive);
    } catch (Exception e) {
      System.err
          .println("DriveSub: Error: PathPlanner failed to initialize! Autos may not work properly. Stack trace:");
      e.printStackTrace();
    }
  }

  private short logRateCounter = 0;
  @Override
  public void periodic() {
    updateOdometry();
    
    if   (logRateCounter < 10) {logRateCounter++;} //log every 200ms
    else                       {
      logRateCounter = 0;
    
      
      Pose2d pose = getPose();
      LogPoseX.append(pose.getX());
      LogPoseY.append(pose.getY());
      LogPoseYaw.append(pose.getRotation().getRadians());
      LogPoseDriveYaw.append(getDriveHeading().in(Radians));
    }
  }

  /**
   * Updates the robot's odometry.
   * 
   * This should be called every robot tick, in the periodic method.
   */
  private void updateOdometry() {
    poseEstimator.update(Rotation2d.fromDegrees(imu.getAngle()), Subsystems.drive.getModulePositions());
    for (final var cam : photon.cams) {
      cam.getEstimatedPose()
          .ifPresent(
            (visionResult) -> {
            final var visionPose = visionResult.estimatedPose.toPose2d();
            poseEstimator.addVisionMeasurement(visionPose, visionResult.timestampSeconds);
          });
    }
    
    
    //normaliseOdometry(); //broken
    
    field.setRobotPose(getPose());
  }

  /** @return the currently estimated pose of the robot. */
  public Pose2d getPose() {

    return normaliseOdometry(poseEstimator.getEstimatedPosition());
  }

  /**
   * Normalises odometry so that the robot's pose is within field bounds.
   */
  public Pose2d normaliseOdometry(Pose2d pose) {
    Translation2d currentLoc = pose.getTranslation();
    Translation2d minPose = DriveConstants.FIELD_BOUNDS[0];
    Translation2d maxPose = DriveConstants.FIELD_BOUNDS[1];
    currentLoc = new Translation2d(
        Math.max(Math.min(currentLoc.getX(), maxPose.getX()), minPose.getX()),
        Math.max(Math.min(currentLoc.getY(), maxPose.getY()), minPose.getY()));
    return new Pose2d(currentLoc, pose.getRotation());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry. If `null`, picks a default
   *             pose depending on alliance color.
   */
  public void resetOdometry(Pose2d pose) {
    // fun fact: PathPlanner can pass null when an auto includes no move command
    if (pose == null) {
      final var onRedAlliance = DriverStation.getAlliance()
          .map(alliance -> alliance == Alliance.Red)
          .orElse(false);

      pose = onRedAlliance ? new Pose2d(1, 4, Rotation2d.kZero) : new Pose2d(15, 4, Rotation2d.k180deg);
    }

    if (Robot.isSimulation()){
      simImuSetAngleYaw(pose.getRotation().getDegrees());
    }

    poseEstimator.resetPosition(Rotation2d.fromDegrees(imu.getAngle()), Subsystems.drive.getModulePositions(), pose);
    
    if (Robot.isSimulation()) {
      simImuSetAngleYaw(pose.getRotation().getDegrees());
      poseSim = pose;
      return;
    }
    
    IMURawOffset = Radians.of(pose.getRotation().getRadians()).minus(getIMUHeading());
  }

  /** Zeroes the Drive heading of the robot returned by getDriveHeading to current angle. */
  public void zeroDriveHeading() {
    IMURawOffset = Radians.of(0).minus(getIMUHeading());
  }

  /** Zeroes the IMU heading of the robot. */
  public void zeroIMUHeading() {
    imu.reset();
  }

  /**
   * @return the robot's field-relative heading according to the pose estimator
   */
  public Angle getHeading() {
    return Radians.of(poseEstimator.getEstimatedPosition().getRotation().getRadians());
  }

  /** @return the robot's heading according to the IMU
   * plus an offset set by reset Odometry to align it with pose estimator angle*/
  public Angle getDriveHeading() {
    return getIMUHeading().plus(IMURawOffset);
  }
  /** @return the robot's heading according to the IMU */
  public Angle getIMUHeading() {
    return Degrees.of(imu.getAngle());
  }

  /** @return the turn rate of the robot */
  public AngularVelocity getTurnRate() {
    return DegreesPerSecond.of(imu.getRate());
  }

  /**
   * @return the robot's heading as a {@link Rotation2d} (direction the robot is
   *         pointing field rel)
   */
  public Rotation2d getHeadingR2D() {
    return Rotation2d.fromRadians(getHeading().in(Radians));
  }

  /** @return the current robot-relative {@link ChassisSpeeds} */
  public ChassisSpeeds getChassisSpeeds() {
    // if (Robot.isSimulation()) {
    // return getDesiredChassisSpeeds();
    // } // modules are not sim'd correctly
    return DriveConstants.DRIVE_KINEMATICS.toChassisSpeeds(Subsystems.drive.getModuleStates());
  }

  /** @return the desired robot-relative {@link ChassisSpeeds} */
  public ChassisSpeeds getDesiredChassisSpeeds() {
    return DriveConstants.DRIVE_KINEMATICS.toChassisSpeeds(Subsystems.drive.getModuleDesiredStates());
  }

  /**
   * @return the current translational speed of the robot (angle irrelevant) (m/s)
   */
  public double getTranslationSpeed() {
    final var speeds = getChassisSpeeds();
    return Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
  }

  public void drawFieldObject(String key, Pose2d position, boolean robotRelative){
    if (robotRelative){
      position = getPose().plus(new Transform2d(position.getTranslation(), position.getRotation()));
    }
    field.getObject(key).setPose(position);
  }

  Translation2d simulationPeriodicLastRobotLocal = new Translation2d();
  double simulationPeriodicLastVel = 0;
  @Override
  public void simulationPeriodic() {
    final var speeds = getDesiredChassisSpeeds();

    simImuSetRateYaw(Math.toDegrees(speeds.omegaRadiansPerSecond));
    // imuSim.setGyroAngleZ(getHeading().plus(Rotation2d.fromRadians(speeds.omegaRadiansPerSecond
    // * 0.02)).getDegrees());
    simImuSetAngleYaw(poseSim.getRotation().getDegrees());

    Translation2d pos = field.getRobotObject().getPose().getTranslation();
    Translation2d tmpPos = pos.minus(simulationPeriodicLastRobotLocal);
    double vel = Math.hypot(tmpPos.getX(), tmpPos.getY()) * 50; // 50 updates per sec
    simImuSetLinAccel((vel - simulationPeriodicLastVel) * 50);
    simulationPeriodicLastRobotLocal = pos;
    simulationPeriodicLastVel = vel;

    poseSim = poseSim.exp(
        new Twist2d(
            speeds.vxMetersPerSecond * 0.02,
            speeds.vyMetersPerSecond * 0.02,
            speeds.omegaRadiansPerSecond * 0.02));
    Translation2d minPose = DriveConstants.FIELD_BOUNDS[0];
    Translation2d maxPose = DriveConstants.FIELD_BOUNDS[1];
    poseSim = new Pose2d(new Translation2d(
        Math.max(Math.min(poseSim.getX(), maxPose.getX()), minPose.getX()),
        Math.max(Math.min(poseSim.getY(), maxPose.getY()), minPose.getY())),
        poseSim.getRotation());

    photon.simulationPeriodic(poseSim);
  }

  private void simImuSetAngleYaw(double angle) {
    switch (imu.getYawAxis()) {
      case kX:
        imuSim.setGyroAngleX(angle);
        break;
      case kY:
        imuSim.setGyroAngleY(angle);
        break;
      case kZ:
        imuSim.setGyroAngleZ(angle);
        break;
      case kRoll:
        imuSim.setGyroAngleX(angle);
        break;
      case kPitch:
        imuSim.setGyroAngleY(angle);
        break;
      case kYaw:
        imuSim.setGyroAngleZ(angle);
        break;
    }
  }

  private void simImuSetRateYaw(double angle) {
    switch (imu.getYawAxis()) {
      case kX:
        imuSim.setGyroRateX(angle);
        break;
      case kY:
        imuSim.setGyroRateY(angle);
        break;
      case kZ:
        imuSim.setGyroRateZ(angle);
        break;
      case kRoll:
        imuSim.setGyroRateX(angle);
        break;
      case kPitch:
        imuSim.setGyroRateY(angle);
        break;
      case kYaw:
        imuSim.setGyroRateZ(angle);
        break;
    }
  }

  // sim accel is broken
  Translation3d simAccel = new Translation3d();

  private void simImuSetLinAccel(double accel) {// TODO improve
    imuSim.setAccelX(accel);
    simAccel = new Translation3d(accel, 0, 0);
  }
}