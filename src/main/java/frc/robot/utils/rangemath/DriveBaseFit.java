package frc.robot.utils.rangemath;

public class DriveBaseFit {

  protected AxesFit axesLin;
  protected AxesFit axesYaw;

  private boolean invertX = false;
  private boolean invertY = false;
  private boolean invertYaw = false;

  protected double ModifierTurnRateBySpeed;

  // -------------------------- Constructor ------------------------

  public DriveBaseFit(AxesFit axesLin, AxesFit axesYaw) {
    this(axesLin, axesYaw, 0);

  }

  public DriveBaseFit(AxesFit axesLin, AxesFit axesYaw, double ModifierTurnRateBySpeed) {
    this.axesLin = axesLin;
    this.axesYaw = axesYaw;
    this.ModifierTurnRateBySpeed = ModifierTurnRateBySpeed;
  }

  /**
   * ---Decorator for the main constructor---
   * <p>
   * invert the input x axis
   */
  public DriveBaseFit invertX() {
    invertX = true;
    return this;
  }

  /**
   * ---Decorator for the main constructor---
   * <p>
   * inverts the input y axis [swerve only]
   */
  public DriveBaseFit invertY() {
    invertY = true;
    return this;
  }

  /**
   * ---Decorator for the main constructor---
   * <p>
   * inverts the yaw axis
   */
  public DriveBaseFit invertYaw() {
    invertYaw = true;
    return this;
  }

  // -------------------------- User functions ------------------------

  /**
   * apply the specified curve to the provided input and returns (no boost, no
   * limit)
   */
  public double[] fitTank(double x, double yaw) {
    return fitTank(x, yaw, 0, 0);
  }

  /**
   * apply the specified curve to the provided input and returns
   * <p>
   * 
   * Booster - This input axis is a percent linear interpolation between normal
   * max and boosted
   * max (0 = norm, 1=full boost)
   * <p>
   * 
   * Limiter - The second input axis is a percent linear interpolation between
   * boosted max and
   * limited max (0 = norm, 1=full limit)
   * <p>
   */
  public double[] fitTank(double x, double yaw, double boostPercent, double limitPercent) {
    if (invertX) {
      x = -x;
    }
    if (invertYaw) {
      yaw = -yaw;
    }

    x = axesLin.fit(x, boostPercent, limitPercent);
    yaw = axesYaw.fit(yaw, boostPercent, limitPercent);
    yaw *= ((1 - ModifierTurnRateBySpeed) + (ModifierTurnRateBySpeed * (Math.abs(x) / axesLin.outAbsMax)));
    return new double[] { x, 0, yaw };
  }

  /**
   * apply the specified curve to the provided input and returns (no boost, no
   * limit)
   */
  public double[] fitSwerve(double x, double y, double yaw) {
    return fitSwerve(x, y, yaw, 0, 0);
  }

  /**
   * apply the specified curve to the provided input and returns
   * <p>
   * 
   * Booster - This input axis is a percent linear interpolation between normal
   * max and boosted
   * max (0 = norm, 1=full boost)
   * <p>
   * 
   * Limiter - The second input axis is a percent linear interpolation between
   * boosted max and
   * limited max (0 = norm, 1=full limit)
   * <p>
   */
  public double[] fitSwerve(double x, double y, double yaw, double boostPercent, double limitPercent) {
    if (invertX) {
      x = -x;
    }
    if (invertY) {
      y = -y;
    }
    if (invertYaw) {
      yaw = -yaw;
    }

    double angle = Math.atan2(y, x);
    double power = Math.hypot(x, y);


    power = axesLin.fit(power, boostPercent, limitPercent);
    yaw = axesYaw.fit(yaw, boostPercent, limitPercent);

    return new double[] { power * Math.cos(angle), power * Math.sin(angle), yaw };
  }
}
