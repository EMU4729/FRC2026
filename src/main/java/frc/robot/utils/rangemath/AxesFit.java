package frc.robot.utils.rangemath;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;

public class AxesFit {
  protected double inMin = -1; // lowest allowed input (clamped)
  protected double inMax = 1; // largest allowed input (clamped)
  protected double outAbsMin = 0; // lowest allowed output once outside the deadband (used to start non pid
                                  // drives)
  protected double outAbsMax = 1; // max allowed output when not limited or boosted
  protected double pow = 1; // A input^pow power applied to any output to increase low speed control. will
                            // work with any >1 input.
  protected double deadband = 0; // The deadband applied to any axes input
  protected boolean invert = false; // invert this axes
  // the new max output when fully boosted (applied before limiter, increases the
  // normal max)
  protected Optional<Double> boosterMax = Optional.empty();
  // the new max output when fully limited (applied after boost, decreases the
  // boosted max)
  protected Optional<Double> limiterMax = Optional.empty();

  // -------------------------- Constructor ------------------------

  /**
   * creates a 'fit' object with default settigns
   * <p>
   * ---[Decorators - use these to select settings]---
   * <p>
   * 
   * withInputMinMax - changes the expected range of inputs (def -1 -> 1)(min <
   * max)
   * <p>
   * withOutputMinMax - changes the range of outputs returned
   * (-absMax -> -absMin [jump] absMin -> absMax) (def min=0,max=1)(0 < min < max)
   * <p>
   * withPow - changes power of the curve applied (def 1)(pow > 1)
   * <p>
   * withDeadBand - changes the deadband applied to the input (def 0)(db >= 0)
   * <p>
   * inverted - negates the provided input (def F)
   * <p>
   * withBooster - allows a reduced normal speed with a second input to increase
   * that limit to a boosted max (def N/A)
   * <p>
   * withLimiter - allows an increase normal speed with a second input to decrease
   * that limit to a limited max (def N/A)(applicable with boost, overrides)
   */
  public AxesFit() {
  }

  /**
   * ---Decorator for the main constructor---
   * <p>
   * Changes the expected continuous range of inputs, outliers are clamped
   * 
   * @param inMin (-1) the minimum value expected
   * @param inMax (1) the max value expected
   */
  public AxesFit withInputMinMax(double inMin, double inMax) {
    if (inMin >= inMax) {
      throw new IllegalArgumentException("inMin >= inMax");
    }
    this.inMin = inMin;
    this.inMax = inMax;
    return this;
  }

  /**
   * ---Decorator for the main constructor---
   * <p>
   * Changes the output range returned. This (is/can be) a discontinous range
   * about 0.
   * Think about a normal custom range from -1 to 1. These values decide how that
   * is stretched
   * before being returned to you.
   * 
   * @param outMin (0) This value decides how input values close to 0 will be
   *               shifted away from 0
   *               in output. So any value less than 0 approaches -outMin as if it
   *               was 0, jumps to 0, jumps
   *               to +outMin, and departs as if +outMin was 0. This allows for a
   *               set starting power so your
   *               robot starts at min throttle and doesn't waste most of your
   *               control range starting.
   * @param outMax (1) This value decides how large an output is returned. The
   *               output range is
   *               stretched to fit this range.
   */
  public AxesFit withOutputMinMax(double outMin, double outMax) {
    if (outMin < 0) {
      throw new IllegalArgumentException("outMin < 0");
    }
    if (outMax < 0) {
      throw new IllegalArgumentException("outMax < 0");
    }
    if (Math.abs(outMin) >= Math.abs(outMax)) {
      throw new IllegalArgumentException("|outMin| >= |outMax|");
    }
    this.outAbsMin = outMin;
    this.outAbsMax = outMax;
    return this;
  }

  /**
   * ---Decorator for the main constructor---
   * <p>
   * The power used in the curve applied to the input. This curve is adjusted to
   * be the same in
   * pos and neg. A larger pow will apply more of the input range to lower output
   * values.
   * 
   * @param pow (1) The input^pow power used for the curve. Must be greater than
   *            1.
   */
  public AxesFit withPow(double pow) {
    this.pow = Math.max(pow, 1);
    return this;
  }

  /**
   * ---Decorator for the main constructor---
   * <p>
   * Analog sticks are made cheap, when the stick is released it may not go to 0.
   * The deadband is
   * the minimum value that will not be equal to 0.
   * 
   * @param deadband (0) Any value (abs) less than this will be set to 0.
   */
  public AxesFit withDeadBand(double deadband) {
    this.deadband = Math.max(deadband, 0);
    return this;
  }

  /**
   * Specifies that the input should be negated before being returned (false)
   */
  public AxesFit inverted() {
    invert = true;
    return this;
  }

  /**
   * As motors are getting more powerful there is a difference between normal
   * driving power and
   * max speed cross the field power. The booster allows some of this power to be
   * locked behind
   * another analog input. So the normal output may max out at 0.7, but when
   * boosted the stick will
   * max out at 0.7 to 1 depending on that second axis for example.
   * <p>
   * 
   * The second input axis is a percent linear interpolation between normal max
   * and boosted max
   * (0 = norm, 1=full boost)
   * <p>
   * 
   * This is applied before the limiter and the resulting boosted max will be used
   * as its
   * non-limited max.
   * <p>
   * 
   * @param boosterMax The new absMax output when fully boosted.
   */
  public AxesFit withBooster(double boosterMax) {
    this.boosterMax = Optional.of(boosterMax);
    return this;
  }

  /**
   * When near game pieces or field elements the drivers need more control to line
   * up quickly. The
   * limiter allows max power to be reduced via another analog input. So the
   * normal output may max
   * out at 0.7, but when fully limited the stick will max out at 0.7 to 0.4
   * depending on that
   * second axis for example.
   * <p>
   * 
   * The second input axis is a percent linear interpolation between normal max
   * and limited max
   * (0 = norm, 1=full limit)
   * <p>
   * 
   * This is applied after the booster and the resulting boosted max is used as
   * its
   * non-limited max.
   * <p>
   * 
   * @param boosterMax The new absMax output when fully boosted.
   */
  public AxesFit withLimiter(double limiterMax) {
    this.limiterMax = Optional.of(limiterMax);
    return this;
  }

  // -------------------------- User functions ------------------------

  /**
   * apply the specified curve to the provided input and returns (no boost, no
   * limit)
   */
  public double fit(double axesInput) {
    return fit(axesInput, 0, 0);
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
  public double fit(double axesInput, double boostPercent, double limitPercent) {
    // invert if needed
    if (invert) {
      axesInput = -axesInput;
    }

    // adjust inputs to a standard range of -1 -> 1
    // this helps with simplifying the math
    axesInput = normaliseInputRange(axesInput);
    axesInput = MathUtil.clamp(axesInput, -1, 1);

    // absolute the input and save the sign for later
    // this makes deadbanding and curving easier as it only needs to be consistent
    // for pos numbers
    double sign = axesInput;
    axesInput = Math.abs(axesInput);

    // deadband by setting <=db to 0 and rerange >db to be from 0 rather than from
    // db
    // this improves latter functions as they will not be truncated at low values by
    // the db
    axesInput = deadband(axesInput);

    // apply a curve to inputs to increase slow speed control
    // the impact of 1 unit of speed is greater when going slow
    // drivers also want more control when going slow
    // we thus dedecate most of the control range to these slower speeds
    axesInput = Math.pow(axesInput, pow);

    // double purpose
    // 1) rerange the values back to a specified output range
    // 2) manage the boost and limit by changing that range
    // getModifiedMax thus calcs the new upper bound based on the cur limit and
    // boost percents
    axesInput = setOutRange(axesInput, outAbsMin, getModifiedMax(boostPercent, limitPercent));

    // replaces the sign that was removed earlier
    axesInput = Math.copySign(axesInput, sign);

    return axesInput;
  }

  // -------------------------- Helper functions ------------------------

  // you will see this (x + a)/(c + d) function a several times past this point.
  // This is a modified line slope function that by using different params
  // for a,b,c will re-range an input by changing the slope between in and out.
  // Primarily, if a=c, a controls what input results in out=0, and b controls
  // out=1.
  // So any->any becomes 0->1.

  private double deadband(double input) {
    // yes i need the custom deadband function
    if (deadband == 0) {
      return input;
    }
    if (input > deadband) {
      return (input - deadband) / (1 - deadband);// re-range back to 0->1
    } else {
      return 0;
    }
  }

  /*
   * converts the range from inMin -> inMax to -1 -> 1
   * the math is easier in this range
   */
  private double normaliseInputRange(double input) {
    if (inMin == -1 && inMax == 1) {
      return input;
    }
    // -inMin : shifts the range up to be 0 -> min+max
    // /(inMax-inMin) : squash the result into 0->1
    // 2*x - 1 : expand and shift down to -1 -> 1
    return 2 * ((input - inMin) / (inMax - inMin)) - 1;
  }

  /** reranges the input to the output range */
  private double setOutRange(double input, double absMin, double absMax) {
    // As before this function reranges the input into the output.
    // the modified version though changes the output range instead of the input
    // So the same 0->1 input becomes any->any
    return (input * (-absMin + absMax)) + absMin;
  }

  /** gets the absolute max output once modified by boost and limiter */
  private double getModifiedMax(double boostPercent, double limitPercent) {
    double runningMax = outAbsMax;

    // if using the booster
    if (boosterMax.isPresent() && boostPercent != 0) {
      boostPercent = MathUtil.clamp(boostPercent, 0, 1);
      // increase the max output linearly between the normal max and fully boosted max
      runningMax += boostPercent * (boosterMax.get() - runningMax);
    }

    // if using the limiter
    if (limiterMax.isPresent() && limitPercent != 0) {
      limitPercent = MathUtil.clamp(limitPercent, 0, 1);
      // decrease the max linearly between the already boosted max and fully limited
      // max
      runningMax -= limitPercent * (runningMax - limiterMax.get());
    }

    return runningMax;
  }
}