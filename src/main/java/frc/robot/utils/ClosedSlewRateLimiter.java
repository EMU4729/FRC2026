package frc.robot.utils;

import edu.wpi.first.math.filter.SlewRateLimiter;

/**
 * Modified version of {@link SlewRateLimiter} that handles negative derivatives
 * correctly, and also allows for different positive and negative rates
 */
public class ClosedSlewRateLimiter {
  private final double positiveRate;
  private final double negativeRate;

  /**
   * Constructs a new ClosedSlewRateLimiter
   * 
   * @param rate The rate at which to limit the value
   */
  public ClosedSlewRateLimiter(double rate) {
    this(rate, -rate);
  }

  /**
   * Constructs a new ClosedSlewRateLimiter
   * 
   * @param positiveRate The rate at which to limit the value when its increasing
   * @param negativeRate The rate at which to limit the value when its decreasing
   *                     - must be negative
   */
  public ClosedSlewRateLimiter(double positiveRate, double negativeRate) {
    this.positiveRate = positiveRate;
    this.negativeRate = negativeRate;
  }

  public double calculate(double desired, double current) {
    if (current == desired) {
      return desired;
    } else if (desired > current) {
      return Math.min(current + positiveRate, desired);
    } else {
      return Math.max(current + negativeRate, desired); // neg rates sign is used specified
    }
  }

}
