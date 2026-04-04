package frc.robot.commands.AutoAlign;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;
import frc.robot.constants.AutoAlignConstants;
import frc.robot.subsystems.DriveSub;


public class AlignToReefTagRelative extends Command {
  private PIDController xController, yController, rotController;
  private boolean isRightScore;
  private Timer dontSeeTagTimer, stopTimer;
  private DriveSub drivebase;
  private double tagID = -1;

  public AlignToReefTagRelative(boolean isRightScore, DriveSub drivebase) {
    xController = new PIDController(AutoAlignConstants.X_REEF_ALIGNMENT_P, 0.0, 0);  // Vertical movement
    yController = new PIDController(AutoAlignConstants.Y_REEF_ALIGNMENT_P, 0.0, 0);  // Horitontal movement
    rotController = new PIDController(AutoAlignConstants.ROT_REEF_ALIGNMENT_P, 0, 0);  // Rotation
    this.isRightScore = isRightScore;
    this.drivebase = drivebase;
    addRequirements(drivebase);
  }

  @Override
  public void initialize() {
    this.stopTimer = new Timer();
    this.stopTimer.start();
    this.dontSeeTagTimer = new Timer();
    this.dontSeeTagTimer.start();

    rotController.setSetpoint(AutoAlignConstants.ROT_SETPOINT_HUB_ALIGNMENT);
    rotController.setTolerance(AutoAlignConstants.ROT_TOLERANCE_HUB_ALIGNMENT);

    xController.setSetpoint(AutoAlignConstants.X_SETPOINT_REEF_ALIGNMENT);
    xController.setTolerance(AutoAlignConstants.X_TOLERANCE_REEF_ALIGNMENT);

    yController.setSetpoint(isRightScore ? AutoAlignConstants.Y_SETPOINT_REEF_ALIGNMENT : -AutoAlignConstants.Y_SETPOINT_REEF_ALIGNMENT);
    yController.setTolerance(AutoAlignConstants.Y_TOLERANCE_REEF_ALIGNMENT);

    // Instead of:  tagID = LimelightHelpers.getFiducialID("");
tagID = Subsystems.nav.photon.getBestVisibleFiducialID();
  }

  @Override
  public void execute() {
    int currentID = Subsystems.nav.photon.getBestVisibleFiducialID();
    if (currentID != -1 && currentID == tagID) {
      this.dontSeeTagTimer.reset();

      double[] postions = Subsystems.nav.photon.getBotPoseTargetSpace();
      SmartDashboard.putNumber("x", postions[2]);

      double xSpeed = xController.calculate(postions[2]);
      SmartDashboard.putNumber("xspee", xSpeed);
      double ySpeed = -yController.calculate(postions[0]);
      double rotValue = -rotController.calculate(postions[4]);

      drivebase.drive(new ChassisSpeeds(xSpeed, ySpeed, rotValue),false, true);

      if (!rotController.atSetpoint() ||
          !yController.atSetpoint() ||
          !xController.atSetpoint()) {
        stopTimer.reset();
      }
    } else {
      drivebase.drive(new ChassisSpeeds(), false, true);
    }

    SmartDashboard.putNumber("poseValidTimer", stopTimer.get());
  }

  @Override
  public void end(boolean interrupted) {
    drivebase.drive(new ChassisSpeeds(), false, false);
  }

  @Override
  public boolean isFinished() {
    // Requires the robot to stay in the correct position for 0.3 seconds, as long as it gets a tag in the camera
    return this.dontSeeTagTimer.hasElapsed(AutoAlignConstants.DONT_SEE_TAG_WAIT_TIME) ||
        stopTimer.hasElapsed(AutoAlignConstants.POSE_VALIDATION_TIME);
  }
}