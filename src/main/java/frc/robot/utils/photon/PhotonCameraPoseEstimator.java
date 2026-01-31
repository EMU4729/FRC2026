package frc.robot.utils.photon;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import java.util.Optional;

import javax.xml.crypto.dsig.Transform;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Subsystems;
import frc.robot.constants.DriveConstants;

public class PhotonCameraPoseEstimator {
  private final PhotonCamera cam;
  public final PhotonCameraSim camSim;
  private final PhotonPoseEstimator poseEstimator;

  private static final double distanceTolBase = 0.2;
  private static final double distanceTolStep = 0.002/2; // *50 for per second increase 
  private static double distanceTol = distanceTolBase;
  private static final double heightTolBase = 0.2;
  private static final double heightTolStep = 0/2;
  private static double heightTol = heightTolBase;
  private final Transform3d robotToCam;
  private final Optional<Transform3d> turretToCam;

  private static final Angle pitchTol = Degrees.of(15);
  private static final Angle rolTol = Degrees.of(15);

  private DoubleLogEntry LogPoseX;
  private DoubleLogEntry LogPoseY;
  private DoubleLogEntry LogPoseZ;
  private DoubleLogEntry LogPoseYaw;
  private StringLogEntry LogTargets;
  private StringLogEntry LogFiltered;

  public PhotonCameraPoseEstimator(
      String cameraName,
      Transform3d robotToCam,
      AprilTagFieldLayout fieldLayout,
      SimCameraProperties camProps) {
        this(cameraName, robotToCam,Optional.empty(), fieldLayout, camProps);
      }

  public PhotonCameraPoseEstimator(
      String cameraName,
      Transform3d robotToCam,
      Optional<Transform3d> turretToCam,
      AprilTagFieldLayout fieldLayout,
      SimCameraProperties camProps) {
  
    DataLog datalog = DataLogManager.getLog();
    LogPoseX = new DoubleLogEntry(datalog, "PhotonPrediction/".concat(cameraName.concat("/X")));
    LogPoseY = new DoubleLogEntry(datalog, "PhotonPrediction/".concat(cameraName.concat("/Y")));
    LogPoseZ = new DoubleLogEntry(datalog, "PhotonPrediction/".concat(cameraName.concat("/Z")));
    LogPoseYaw = new DoubleLogEntry(datalog, "PhotonPrediction/".concat(cameraName.concat("/Yaw")));
    LogTargets = new StringLogEntry(datalog, "PhotonPrediction/".concat(cameraName.concat("/Targets")));
    LogFiltered = new StringLogEntry(datalog, "PhotonPrediction/".concat(cameraName.concat("/FilterResult")));

        this.robotToCam = robotToCam;
        this.turretToCam = turretToCam;
        
    cam = new PhotonCamera(cameraName);
    camSim = new PhotonCameraSim(cam, camProps);
    camSim.enableProcessedStream(RobotBase.isSimulation());

    poseEstimator = new PhotonPoseEstimator(
        fieldLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        robotToCam);
      
    poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.AVERAGE_BEST_TARGETS);
  }

  /** @return the robot to camera transform for this camera */
  public Transform3d getRobotToCameraTransform() {
    return poseEstimator.getRobotToCameraTransform();
  }

  /**
   * Updates the {@link PhotonPoseEstimator}'s pose estimation with the latest
   * vision result. Should be called once per robot tick.
   * 
   * @return the new currently estimated pose
   */

    /**
   * Updates the {@link PhotonPoseEstimator}'s pose estimation with the latest
   * vision result. Should be called once per robot tick.
   * 
   * @return the new currently estimated pose
   */
    public Optional<EstimatedRobotPose> getEstimatedPose() {
          return getEstimatedPose(new Rotation3d());
    }

    public Optional<EstimatedRobotPose> getEstimatedPose(Rotation3d turretAngle) {
    Optional<PhotonPipelineResult> latestResult = getLatestResult();

    if (latestResult.isEmpty()) return Optional.empty();
    Transform3d robotToCamTemp = robotToCam;
    if (turretToCam.isPresent()) {
      Transform3d rotationTransform = new Transform3d(new Translation3d(), turretAngle);
      Transform3d turretTotransform = turretToCam.get().plus(rotationTransform);
      robotToCamTemp = turretToCam.get().plus(turretTotransform);
    }
    poseEstimator.setRobotToCameraTransform(robotToCamTemp);


    Optional<EstimatedRobotPose> res = filter(log(latestResult.flatMap(poseEstimator::update)));
    if (res.isEmpty()){increaseTollerance();}
    else {decreaseTollerance();}

    return res;
  }

  public double getDistanceTol(){
    return distanceTol;
  }

  private boolean wasConnected = true;
  /** @return the latest vision result from the camera */
  public Optional<PhotonPipelineResult> getLatestResult() {
    if (!cam.isConnected()) {
      if(wasConnected){
        DataLogManager.log("PhotonBridge: Error: Camera not connected");
      }
      return Optional.empty();
    } else {
      if(!wasConnected){
        DataLogManager.log("PhotonBridge: Info: Camera reconnected");
      }
    }
    wasConnected = cam.isConnected();

    final var results = cam.getAllUnreadResults();
    if (results.isEmpty()) {
      return Optional.empty();
    }
    // is the latest result at the end or the beginning??
    final var latestResult = results.get(results.size() - 1);
    return Optional.of(latestResult);
  }

  /** Resets the internal pose to an origin pose */
  public void reset() {
    reset(new Pose2d());
  }

  public Optional<EstimatedRobotPose> filter(Optional<EstimatedRobotPose> OptPose){
    if (OptPose.isEmpty()) return OptPose;
    EstimatedRobotPose pose = OptPose.get();
    Pose2d currentPose = Subsystems.nav.getPose();
    double distanceError = pose.estimatedPose
        .getTranslation()
        .toTranslation2d()
        .getDistance(currentPose.getTranslation());
    double heightError = pose.estimatedPose.getZ();

    if ( distanceError > distanceTol) {
      LogFiltered.append("Distance");
      return Optional.empty();
    }
    if ( Math.abs(heightError) > heightTol) {
      LogFiltered.append("Height"); 
      return Optional.empty();
    }
    if ( pose.estimatedPose.getX() < DriveConstants.FIELD_BOUNDS[0].getX() || 
        pose.estimatedPose.getX() >  DriveConstants.FIELD_BOUNDS[1].getX() ||
        pose.estimatedPose.getY() <  DriveConstants.FIELD_BOUNDS[0].getY() ||
        pose.estimatedPose.getY() >  DriveConstants.FIELD_BOUNDS[1].getY()){
          LogFiltered.append("Field Bounds"); 
          return Optional.empty();
        }
    if ( Math.abs(pose.estimatedPose.getRotation().getX()) > rolTol.in(Radians)){
      LogFiltered.append("Roll");  
      return Optional.empty();
    }
    if ( Math.abs(pose.estimatedPose.getRotation().getY()) > pitchTol.in(Radians)){
      LogFiltered.append("Pitch");
      return Optional.empty();
    }

    LogFiltered.append("Passed");
    return OptPose;
  }
  public static void increaseTollerance(){
    distanceTol = distanceTol+distanceTolStep;
    heightTol   = heightTol+heightTolStep;
    
  }
  public static void decreaseTollerance(){
    distanceTol = Math.max(distanceTol-5*distanceTolStep, distanceTolBase);
    heightTol   = Math.max(heightTol-5*heightTolStep, heightTolBase);

  }

  private Optional<EstimatedRobotPose> log(Optional<EstimatedRobotPose> OptPose){
    OptPose.ifPresent((pose)->{
      LogPoseX.append(pose.estimatedPose.getX());
      LogPoseY.append(pose.estimatedPose.getY());
      LogPoseZ.append(pose.estimatedPose.getZ());
      LogPoseYaw.append(pose.estimatedPose.getRotation().getZ());

      String targets = "TargetIds:[";
      Boolean first = true;
      for(PhotonTrackedTarget a : pose.targetsUsed){
        if (!first){targets = targets.concat(", ");}
        targets = targets.concat(Integer.toString(a.fiducialId));
        first = false;
      }
      targets = targets.concat("]");
      LogTargets.append(targets);
    });
    return OptPose;
  }

  /**
   * Resets the internal pose
   * 
   * @param pose The pose to reset to
   */
  public void reset(Pose2d pose) {
    poseEstimator.setLastPose(pose);
    poseEstimator.setReferencePose(pose);
  }
}