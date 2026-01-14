package frc.robot.commands.utility;

import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/**
 * A command that extends {@link WaitCommand} with a progress bar, accessible
 * via SmartDashboard
 */
public class WaitProgressCommand extends WaitCommand {
  private final String key;
  private final Time duration;

  /**
   * Constructs a new {@link WaitProgressCommand}
   * 
   * @param key      The SmartDashboard key to publish progress to
   * @param duration The duration of the waiting period
   */
  public WaitProgressCommand(String key, Time duration) {
    super(duration);
    this.key = key;
    this.duration = duration;
  }

  @Override
  public void execute() {
    super.execute();
    SmartDashboard.putNumber(key, m_timer.get() / duration.in(Second));
  }

  @Override
  public void end(boolean interrupt) {
    super.end(interrupt);
    SmartDashboard.putNumber(key, 0);
  }
}
