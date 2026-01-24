package frc.robot;

import frc.robot.subsystems.DriveSub;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.NavigationSub;
import frc.robot.subsystems.IntakeSub;

/**
 * Subsystems - Use this class to initialize and access all subsystems globally.
 */
public class Subsystems {
  public static final DriveSub drive = new DriveSub();
  public static final NavigationSub nav = new NavigationSub();
  public static final LEDSubsystem led = new LEDSubsystem();
  public static final IntakeSub intake = new IntakeSub();
}
