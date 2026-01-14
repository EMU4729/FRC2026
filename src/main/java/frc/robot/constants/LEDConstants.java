package frc.robot.constants;

import frc.robot.Robot;

public class LEDConstants {
  /** LED string length (in leds) */
  public static final int STRING_LENGTH = 150;
  /** LED string port num */
  public static final int STRING_PORT = 3;

  /**
   * Max number of colour changes/s (red -> black -> red -> black = 4) for leds
   */
  public static final int MAX_FLASH_RATE = 6; // <= 6 please. We dont want to risk hurting someone.

  /** max brightness for long periods (0-255) */
  public static final int MAX_SUSTAINED_BRIGHTNESS = Robot.isReal() ? 20 : 255;
}
