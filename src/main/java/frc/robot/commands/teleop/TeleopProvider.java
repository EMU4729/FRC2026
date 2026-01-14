package frc.robot.commands.teleop;

import java.util.Optional;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems;
import frc.robot.constants.DriveConstants;

/**
 * Provides the default command for teleop.
 */
public class TeleopProvider {
  private static Optional<TeleopProvider> inst = Optional.empty();

  private final Command disableCommand = new InstantCommand(() -> {
  }, Subsystems.drive);
  private final Command teleopSwerve = new TeleopDriveSwerve(DriveConstants.PILOT_SETTINGS);
  private final Command teleopDemoSwerve = new TeleopDriveSwerve(DriveConstants.PILOT_DEMO_SETTINGS);

  private final SendableChooser<Command> chooser = new SendableChooser<>(); // pub for shuffle board

  private TeleopProvider() {
    chooser.setDefaultOption("Disable Teleop", disableCommand);
    chooser.addOption("Swerve Teleop", teleopSwerve);
    chooser.addOption("Swerve Demo Teleop", teleopDemoSwerve);

    chooser.onChange(Subsystems.drive::setDefaultCommand);
    Subsystems.drive.setDefaultCommand(chooser.getSelected()); // set default on startup

    SmartDashboard.putData("Teleop Chooser", chooser);
  }

  /** @return the singleton instance of the provider */
  public static TeleopProvider getInstance() {
    if (!inst.isPresent()) {
      inst = Optional.of(new TeleopProvider());
    }
    return inst.get();
  }

  /** @return the currently selected teleop command */
  public Command getSelected() {
    return chooser.getSelected();
  }
}
