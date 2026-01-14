package frc.robot.commands.analysis;

import java.util.Optional;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;

public class AngularSpeedAnalysis extends Command {
  private final static double MODULE_FREE_SPEED = 5;
  private final SlewRateLimiter accelerationLimiter = new SlewRateLimiter(0.1);
  private double maxSpeed = 0;

  private Optional<DoubleLogEntry> LogFile = Optional.empty();

  public AngularSpeedAnalysis() {


    addRequirements(Subsystems.drive);
  }

  /**
   * sets the swerve to turn on the spot
   * 
   * @param v The speed to turn at
   */
  private void setSpeed(double v) {
    Subsystems.drive.setModuleStates(new SwerveModuleState[] {
        new SwerveModuleState(v, Rotation2d.fromDegrees(-45)),
        new SwerveModuleState(v, Rotation2d.fromDegrees(-135)),
        new SwerveModuleState(v, Rotation2d.fromDegrees(45)),
        new SwerveModuleState(v, Rotation2d.fromDegrees(135)),
    });
  }

  @Override
  public void initialize() {
    DataLogManager.log("Angular Speed Analysis : Started");
    if (LogFile.isEmpty()) {
      LogFile = Optional.of(new DoubleLogEntry(DataLogManager.getLog(), "Analysis/AngularSpeed"));
    }
    super.initialize();
  }

  public void execute() {
    final var v = accelerationLimiter.calculate(MODULE_FREE_SPEED);
    setSpeed(v);

    final var chassisSpeeds = Subsystems.nav.getChassisSpeeds();
    final var speed = Math.abs(chassisSpeeds.omegaRadiansPerSecond);

    SmartDashboard.putNumber("Angular Speed", speed);
    if (speed > maxSpeed) {
      maxSpeed = speed;
      SmartDashboard.putNumber("Max Angular Speed", maxSpeed);
      LogFile.get().append(maxSpeed);
    }
  }

  public void end(boolean interrupted) {
    setSpeed(0);
    accelerationLimiter.reset(0);
    DataLogManager.log("Angular Speed Analysis : Finished");
  }
}
