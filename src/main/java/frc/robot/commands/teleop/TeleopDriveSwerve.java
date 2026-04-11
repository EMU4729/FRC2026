package frc.robot.commands.teleop;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.Optional;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.Subsystems;
import frc.robot.utils.rangemath.DriveBaseFit;
import frc.robot.constants.DriveConstants;

public class TeleopDriveSwerve extends Command {
  private final DriveBaseFit settings;

  public TeleopDriveSwerve(DriveBaseFit settings) {
    this.settings = settings;
    addRequirements(Subsystems.drive);
  }

  @Override
  public void execute() {
    if (!DriverStation.isTeleop())
      return;

      /*
       * Set X wheel position. DIsabled, due to unwilling to change drivebase.
       */
   // boolean setX = OI.pilot.getHID().getYButton();
   // if (setX) {
   //   Subsystems.drive.setX();
   //   return;
   // }

    double limiter = OI.pilot.getRightTriggerAxis();
    double booster = OI.pilot.getHID().getRightBumperButton() ? 1 : 0;
    // boolean fieldRelative = !OI.pilot.getHID().getLeftBumperButton();
    boolean fieldRelative = true;

    final var control = settings.fitSwerve(
        OI.pilot.getLeftY(),
        OI.pilot.getLeftX(),
        -OI.pilot.getRightX(),
        booster,
        limiter);

    SmartDashboard.putNumber("Controller/y", OI.pilot.getLeftY());
    SmartDashboard.putNumber("Controller/x", OI.pilot.getLeftX());
    SmartDashboard.putNumber("Controller/r", -OI.pilot.getRightX());

    var x = -control[0] * DriveConstants.MAX_SPEED.in(MetersPerSecond);
    var y = -control[1] * DriveConstants.MAX_SPEED.in(MetersPerSecond);
    var r = control[2] * DriveConstants.MAX_ANGULAR_SPEED.in(RadiansPerSecond);

    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get().equals(Alliance.Red)) {
      x = -x;
      y = -y;
    }
    ;

    SmartDashboard.putNumber("Controller/xo", x);
    SmartDashboard.putNumber("Controller/yo", y);
    SmartDashboard.putNumber("Controller/ro", r);

    final var speeds = new ChassisSpeeds(x, y, r);
    Subsystems.drive.drive(speeds, fieldRelative, true);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}