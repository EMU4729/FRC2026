package frc.robot;

import frc.robot.subsystems.DriveSub;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.NavigationSub;
import frc.robot.subsystems.TurretAimingSub;
import frc.robot.subsystems.TurretFeederSub;
import frc.robot.subsystems.TurretShooterSub;
import frc.robot.subsystems.IntakeSub;

/**
 * Subsystems - Use this class to initialize and access all subsystems globally.
 */
public class Subsystems {
  public static final DriveSub drive = new DriveSub();
  public static final NavigationSub nav = new NavigationSub();
  public static final LEDSubsystem led = new LEDSubsystem();
  public static final IntakeSub intake = new IntakeSub();
  public static final TurretAimingSub turretAiming = new TurretAimingSub();
  public static final TurretShooterSub turretShooter = new TurretShooterSub();
  public static final TurretFeederSub turretFeeder = new TurretFeederSub();
}
