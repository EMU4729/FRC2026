// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.List;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ActivateIntakeCommand;
import frc.robot.commands.AutoAlign.AlignToReefTagRelative;
import frc.robot.constants.AimingConstants;
import frc.robot.constants.HopperConstants;
import frc.robot.commands.auto.AutoProvider;
import frc.robot.commands.autoaim.AutoAim;
import frc.robot.commands.teleop.TeleopProvider;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including
 * Subsystemsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final AutoProvider autoProvider;
  private final TeleopProvider teleopProvider;

  /**
   * The container for the robot. Contains Subsystemsystems, OI devices, and
   * commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    // NamedCommands.registerCommand("Intake ON", IntakeCommand.forAuto());
    // NamedCommands.registerCommand("HOPPER ON", ActivateHopperCommand.forAuto());
    if (Robot.isSimulation()) {
      DriverStation.silenceJoystickConnectionWarning(true);
    }

    configureButtonBindings();
    autoProvider = AutoProvider.getInstance();
    teleopProvider = TeleopProvider.getInstance();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Robot Automations

    // Flash LEDs yellow on endgame
    new Trigger(() -> DriverStation.isTeleop() && DriverStation.getMatchTime() <= 30).onTrue(
        Subsystems.led.runPattern(LEDPattern.solid(Color.kYellow).blink(Seconds.of(0.5))).withTimeout(2));

    // final var rumbleTimes = List.of(30 - 3, 55 - 3, 80 - 3, 105 - 3);
    // new Trigger(() -> DriverStation.isTeleop() && rumbleTimes.contains((int)
    // DriverStation.getMatchTime()))
    // .onTrue(new SequentialCommandGroup(
    // new InstantCommand(() -> OI.pilot.setRumble(RumbleType.kBothRumble, 0.5)),
    // new WaitCommand(0.5),
    // new InstantCommand(() -> OI.pilot.setRumble(RumbleType.kBothRumble, 0)),
    // new WaitCommand(0.5)).repeatedly().withTimeout(3));

    // +----------------+
    // | PILOT CONTROLS |
    // +----------------+

    // --- Manual Controls ---
    OI.pilot.start()
        .onTrue(new InstantCommand(Subsystems.nav::zeroDriveHeading, Subsystems.drive));

    OI.pilot.leftTrigger().whileTrue(new ActivateIntakeCommand(MetersPerSecond.of(2.5)));

    // Bind pilot Y (north) to IntakeCommand (mirror behavior in Turret package)
    // OI.pilot.y().whileTrue(new
    // ActivateIntakeCommand(MetersPerSecond.of(MOTOR_SPEED)));

    // Bind pilot B (east) to the hopper activation command while held

    Command pulseHopperCommand = new SequentialCommandGroup(
        Subsystems.hopper.runCommand(1).withTimeout(0.2),
        new WaitCommand(0.1))
        .repeatedly();
    OI.pilot.b().whileTrue(pulseHopperCommand);

    // THIS BUTTON IS FOR SHOOTING, TAKE A LOOK AT TURRETRUNNER, FOR OTHER COOKED
    // CONTROLS, FOR SHOOTING
    OI.pilot.a().whileTrue(pulseHopperCommand);// and shoot

    OI.pilot.leftBumper()
        .onTrue(new InstantCommand(() -> Subsystems.intake.setRetractedAngle()))
        .onFalse(new InstantCommand(() -> Subsystems.intake.setExtendAngle()).ignoringDisable(true));
    // .whileTrue(Subsystems.hopper.runCommand(1));

    // THIS IS A REFURBISHED 2025 CODE, FOR 2026. It aligns, both translation and
    // rotation.

    // OI.pilot.rightTrigger()
    // .whileTrue(new AlignToReefTagRelative(true, Subsystems.drive));
    //
    // // Left score - hold Left Bumper
    // OI.pilot.leftTrigger()
    // .whileTrue(new AlignToReefTagRelative(false, Subsystems.drive));

    OI.pilot.x().whileTrue(new AutoAim());

    // OI.pilot.rightTrigger()
    // .whileTrue(new AlignToReefTagRelative(true, Subsystems.drive));
    //
    // // Left score - hold Left Bumper
    // OI.pilot.leftTrigger()
    // .whileTrue(new AlignToReefTagRelative(false, Subsystems.drive));
  }

  /**
   * Use this to pass the teleop command to the main {@link Robot} class.
   *
   * @return the command to run in teleop
   */
  public Command getTeleopCommand() {
    return teleopProvider.getSelected();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoProvider.getSelected();
  }
}
