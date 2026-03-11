package frc.robot.commands.Turret;

import static edu.wpi.first.units.Units.Degrees;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import static edu.wpi.first.units.Units.Radians;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;
import frc.robot.constants.AimingConstants;
import frc.robot.constants.TurretConstants;
import frc.robot.constants.AimingConstants.TurretState;
import frc.robot.utils.TurretAiming;

/**
 * Command: Aim the turret at the best AprilTag in front of the robot.
 *
 * Behavior:
 * - While executed, select the nearest AprilTag that is roughly facing the robot.
 * - Compute turret yaw, hood angle and shooter power via TurretAiming.calcState.
 * - Command TurretAimingSub and TurretShooterSub with the computed targets.
 */
public class TurretAimAtTag extends Command {
    final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    public TurretAimAtTag() {
        addRequirements(Subsystems.turretAiming, Subsystems.turretFeeder, Subsystems.turretShooter);
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
    }

    @Override
    public void execute() {
        Pose2d robotPose = Subsystems.nav.getPose();

        Optional<AprilTag> maybeTag = TurretAiming.getBestTarget(robotPose);

        if (maybeTag.isEmpty()) {
            // No suitable tag found; don't change targets.
            return;
        }

        AprilTag tag = maybeTag.get();
        Translation2d tagPos = tag.pose.getTranslation().toTranslation2d();

        // Draw the target on the field (visual debugging)
        Subsystems.nav.drawFieldObject("TurretTarget", new Pose2d(tagPos, new Rotation2d()), false);

        // Compute turret state (angle, power, hood)
        TurretState state = TurretAiming.calcState(AimingConstants.ShootingSamples, tagPos);

    // Rotate the robot so the front faces the target (simple-shooter mode)
    Subsystems.drive.driveAtAngle(new ChassisSpeeds(0, 0, 0), true,
        Rotation2d.fromRadians(state.turretAngle().in(Radians)));

    // Configure hood and shooter power as before
    Subsystems.turretAiming.setHoodTarget(state.hoodAngle());
    Subsystems.turretShooter.setSpeed(state.power());

        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        // Stop motion and motors when command ends
        Subsystems.drive.setX();
        Subsystems.turretAiming.stop();
        Subsystems.turretFeeder.stop();
        Subsystems.turretShooter.stop();
        super.end(interrupted);
    }
}
