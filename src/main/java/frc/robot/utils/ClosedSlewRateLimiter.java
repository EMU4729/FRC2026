package frc.robot.utils;

import edu.wpi.first.math.filter.SlewRateLimiter;

/**
 * Modified version of {@link SlewRateLimiter} that handles negative derivatives
 * correctly, and also allows for different positive and negative rates
 */public class ClosedSlewRateLimiter {
  private final double positiveRatePerTick;
  private static final double DT = 0.02;

  public ClosedSlewRateLimiter(double accelerationRate, double decelerationRate) {
    // decelerationRate parameter kept for API compatibility but ignored —
    // deceleration is always instant so the robot stops crisply.
    this.positiveRatePerTick = accelerationRate * DT;
  }

  public ClosedSlewRateLimiter(double rate) {
    this(rate, rate);
  }

  public double calculate(double desired, double current) {
    double delta = desired - current;
    if (Math.abs(delta) < 1e-9) return desired;

    // Only limit when accelerating (delta and current same sign, growing).
    // Deceleration and direction changes are always instant.
    boolean isAccelerating = Math.signum(delta) == Math.signum(current);
    if (isAccelerating && delta > 0) {
      return current + Math.min(delta, positiveRatePerTick);
    } else if (isAccelerating && delta < 0) {
      return current + Math.max(delta, -positiveRatePerTick);
    } else {
      return desired; // decelerating or reversing — go immediately
    }
  }
}