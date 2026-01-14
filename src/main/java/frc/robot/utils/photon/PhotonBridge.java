package frc.robot.utils.photon;

import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.constants.VisionConstants;

public class PhotonBridge {
  public final PhotonCameraPoseEstimator[] cams;

  // Simulation
  private VisionSystemSim visionSim;

  public PhotonBridge() {
    final var fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    final var camProps = new SimCameraProperties();
    camProps.setCalibration(
        VisionConstants.CAM_RES_WIDTH,
        VisionConstants.CAM_RES_HEIGHT,
        VisionConstants.CAM_DIAG_FOV);
    camProps.setCalibError(0.25, 0.08);
    camProps.setFPS(45);
    camProps.setAvgLatencyMs(35);
    camProps.setLatencyStdDevMs(5);

    cams = new PhotonCameraPoseEstimator[] {
        new PhotonCameraPoseEstimator(
            VisionConstants.PHOTON_CAMERA_NAME[0],
            VisionConstants.ROBOT_TO_CAMERA[0],
            fieldLayout,
            camProps),

        new PhotonCameraPoseEstimator(
            VisionConstants.PHOTON_CAMERA_NAME[1],
            VisionConstants.ROBOT_TO_CAMERA[1],
            fieldLayout,
            camProps)
    };

    if (RobotBase.isSimulation()) {
      visionSim = new VisionSystemSim(VisionConstants.PHOTON_CAMERA_NAME[0]);
      visionSim.addAprilTags(fieldLayout);
      for (final var cam : cams) {
        visionSim.addCamera(cam.camSim, cam.getRobotToCameraTransform());
      }
    }
  }

  private Transform2d simError = new Transform2d();

  public void simulationPeriodic(Pose2d pose) {
    if (visionSim == null)
      return;

    // introduce random error into simulation pose to simulate real-world vision
    // conditions
    simError.plus(new Transform2d(
        (Math.random() - 0.5) * 2, (Math.random() - 0.5) * 2,
        Rotation2d.fromDegrees((Math.random() - 0.5) * 10)));
    visionSim.update(pose.plus(simError));
  }
}
