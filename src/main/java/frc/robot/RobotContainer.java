// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ActivateIntakeCommand;
import frc.robot.commands.Turret.IntakeCommand;
import frc.robot.commands.ActivateHopperCommand;
import frc.robot.constants.HopperConstants;
import frc.robot.commands.auto.AutoProvider;
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
    if(Robot.isSimulation()){
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

    // +----------------+
    // | PILOT CONTROLS |
    // +----------------+

    // --- Manual Controls ---
    OI.pilot.start()
        .onTrue(new InstantCommand(Subsystems.nav::zeroDriveHeading, Subsystems.drive));

    
    OI.pilot.a().whileTrue(new ActivateIntakeCommand(MetersPerSecond.of(ActivateIntakeCommand.MOTOR_SPEED)));

  // Bind pilot Y (north) to IntakeCommand (mirror behavior in Turret package)
  OI.pilot.y().whileTrue(new IntakeCommand(MetersPerSecond.of(ActivateIntakeCommand.MOTOR_SPEED)));

  // Bind pilot B (east) to the hopper activation command while held
  OI.pilot.b().whileTrue(new ActivateHopperCommand(HopperConstants.HOPPER_DEFAULT_SPEED));

  
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
