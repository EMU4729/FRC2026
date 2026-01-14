package frc.robot.commands.auto;

import java.util.Optional;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;
import frc.robot.commands.analysis.AngularSpeedAnalysis;
import frc.robot.commands.analysis.LateralSpeedAnalysis;
import frc.robot.constants.DriveConstants;
import frc.robot.utils.pathplanner.AutoBuilderFix;

/**
 * Provides the default command for autonomous.
 */
public class AutoProvider {
  private static Optional<AutoProvider> inst = Optional.empty();

  private final SendableChooser<Command> chooser;

  private AutoProvider() {
    chooser = new SendableChooser<>(); // pub for shuffle board

    // This is here to ensure PathPlanner is configured before we attempt to call
    // anything AutoBuilder-related, since static classes are lazily constructed.
    // I now see why WPILib's docs recommend dependency-injecting subsystems rather
    // than global static access. - Neel
    @SuppressWarnings("unused")
    final var _nav = Subsystems.nav;

    loadPathPlannerAuto("Left Starting Position", "Left Starting Position");
    loadPathPlannerAuto("Right Starting Position", "Right Starting Position");
    loadPathPlannerAuto("Mid Starting Position", "Mid Starting Position");
    loadPathPlannerAuto("Newtest", "Newtest");

    chooser.addOption(
        "Pathfind to Pose Test",
        AutoBuilderFix.pathfindToPose(new Pose2d(1, 1, Rotation2d.k180deg), DriveConstants.PATH_CONSTRAINTS));
    chooser.addOption("Lateral Speed Analysis", new LateralSpeedAnalysis());
    chooser.addOption("Angular Speed Analysis", new AngularSpeedAnalysis());
    chooser.addOption("System Test", Subsystems.drive.testFunction());

    SmartDashboard.putData("Auto Chooser", chooser);

  }

  /** @return The instance of AutoProvider. */
  public static AutoProvider getInstance() {
    if (!inst.isPresent()) {
      inst = Optional.of(new AutoProvider());
    }
    return inst.get();

  }

  /** @return The selected command. */
  public Command getSelected() {
    return chooser.getSelected();
  }

  /**
   * Loads an auto from PathPlanner into the provider.
   * 
   * @param name The name of the auto in the chooser.
   * @param key  The name of the auto in PathPlanner.
   */
  private void loadPathPlannerAuto(String name, String key) {
    try {
      chooser.addOption(name, new PathPlannerAuto(key));
    } catch (Exception e) {
      System.err.println("Auto Provider : load failed : Name:" + name + ", Key:" + key);
      // e.printStackTrace();
    }
  }
}