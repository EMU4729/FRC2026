package frc.robot.commands.autoaim;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;
import frc.robot.constants.AimingConstants;
import frc.robot.constants.AutoAlignConstants;
import frc.robot.subsystems.DriveSub;
import frc.robot.utils.TurretAiming;

public class AutoAim extends Command {
  // todo tune these
  private final PIDController turnController = new PIDController(3, 0, 0);
  private Translation2d hubPosition;

  public AutoAim() {
    turnController.enableContinuousInput(-Math.PI, Math.PI);
    turnController.setTolerance(Math.PI / 30);
    addRequirements(Subsystems.drive);
  }

  @Override
  public void initialize() {
    // System.out.println("init");
    turnController.reset();
    hubPosition = DriverStation.getAlliance()
        .map((alliance) -> alliance == Alliance.Red
            ? AimingConstants.Red_Hub
            : AimingConstants.Blue_Hub)
        .orElse(AimingConstants.Red_Hub);

  }

  @Override
  public void execute() {
    final var targetTransform = TurretAiming.getTargetTransform(hubPosition);
    System.out.println(targetTransform.toString());
    final var error = targetTransform.getRotation().getRadians();
    final var out = turnController.calculate(error);
    // System.out.println(out);
    // System.out.println(error);
    Subsystems.drive.drive(new ChassisSpeeds(0, 0, out), false, false);
  }

  @Override
  public boolean isFinished() {
    return turnController.atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    Subsystems.drive.drive(new ChassisSpeeds(), false, false);
  }
}