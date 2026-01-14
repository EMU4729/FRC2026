package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Hertz;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.LEDConstants;

public class LEDSubsystem extends SubsystemBase {
  private final AddressableLED leds = new AddressableLED(LEDConstants.STRING_PORT);
  private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(LEDConstants.STRING_LENGTH);

  public LEDSubsystem() {
    leds.setLength(LEDConstants.STRING_LENGTH);
    leds.start();

    setDefaultCommand(runPattern(LEDPattern.rainbow(255, 128).scrollAtRelativeSpeed(Hertz.of(1))));
  }

  /**
   * Returns an command that runs forever that runs the specified pattern.
   * 
   * @param pattern The pattern to run
   * @return The command that will run the pattern
   */
  public Command runPattern(LEDPattern pattern) {
    return run(() -> pattern.applyTo(buffer));
  }
}
