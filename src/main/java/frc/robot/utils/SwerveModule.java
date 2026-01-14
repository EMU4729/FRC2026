package frc.robot.utils;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.Subsystems;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.DriveConstants.SwerveModuleDetails;

public class SwerveModule {
  private final SwerveModuleDetails details;

  private final TalonFX driveMotor;
  private final SparkMax turnMotor;

  private final AbsoluteEncoder turnEncoder;

  private final VelocityVoltage driveController;
  private final SparkClosedLoopController turnController;

  /** the module's desired state, <strong>relative to the module.</strong> */
  private OptimisedSwerveModuleState desiredState = new OptimisedSwerveModuleState(0, new Rotation2d());

  private Distance simPose = Meters.of(0);
  private LinearVelocity simVelocity = MetersPerSecond.of(0);
  private Angle simAngle= Radians.of(0);
  private AngularVelocity simAngularRate = RadiansPerSecond.of(0);

  DoubleLogEntry logVTarg;
  DoubleLogEntry logVTrue;
  DoubleLogEntry logATarg;
  DoubleLogEntry logATrue;
  BooleanLogEntry logOptimised;

  /**
   * Constructs a new SwerveModule for a MAX Swerve Module housing a
   * TalonFX-controlled
   * driving motor and a SparkMax-controlled Turning Motor.
   * 
   * @param moduleDetails the details of the module
   */
  public SwerveModule(SwerveModuleDetails moduleDetails) {
    this.details = moduleDetails;

    // DRIVE MOTOR CONFIG
    driveMotor = new TalonFX(moduleDetails.driveCANID());
    TalonFXConfiguration driveMotorConfig = new TalonFXConfiguration();
    driveMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveMotorConfig.Feedback.SensorToMechanismRatio = DriveConstants.DRIVE_GEAR_RATIO;
    driveMotorConfig.Slot0.kP = DriveConstants.DRIVE_P;
    driveMotorConfig.Slot0.kI = DriveConstants.DRIVE_I;
    driveMotorConfig.Slot0.kD = DriveConstants.DRIVE_D;
    if (moduleDetails.invertDrive()) {
      driveMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    }
    driveMotor.getConfigurator().apply(driveMotorConfig);

    driveController = new VelocityVoltage(0).withFeedForward(DriveConstants.DRIVING_FF).withSlot(0);

    // TURNING MOTOR CONFIG
    SparkMaxConfig turnMotorConfig = new SparkMaxConfig();
    turnMotorConfig.absoluteEncoder
        .positionConversionFactor(DriveConstants.TURNING_ENCODER_POSITION_FACTOR)
        .velocityConversionFactor(DriveConstants.TURNING_ENCODER_VELOCITY_FACTOR);
    turnMotorConfig.closedLoop
        .pidf(
            DriveConstants.TURNING_P,
            DriveConstants.TURNING_I,
            DriveConstants.TURNING_D,
            DriveConstants.TURNING_FF)
        .iZone(DriveConstants.TURNING_I_ZONE.in(Radians))
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(DriveConstants.TURNING_ENCODER_POSITION_PID_MIN_INPUT,
            DriveConstants.TURNING_ENCODER_POSITION_PID_MAX_INPUT);
    turnMotorConfig.idleMode(IdleMode.kBrake);

    turnMotor = new SparkMax(moduleDetails.steerCANID(), MotorType.kBrushless);
    turnMotor.configure(turnMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    turnController = turnMotor.getClosedLoopController();
    turnEncoder = turnMotor.getAbsoluteEncoder();

    // --------------GO TO DEFAULTS--------------
    desiredState.angle = new Rotation2d(turnEncoder.getPosition());
    driveMotor.setPosition(0);



    DataLog datalog = DataLogManager.getLog();
    logVTarg = new DoubleLogEntry(datalog, "SwerveModule/".concat(details.name().concat("/VTarget")));
    logVTrue = new DoubleLogEntry(datalog, "SwerveModule/".concat(details.name().concat("/VActual")));
    logATarg = new DoubleLogEntry(datalog, "SwerveModule/".concat(details.name().concat("/AngleTarget")));
    logATrue = new DoubleLogEntry(datalog, "SwerveModule/".concat(details.name().concat("/AngleActual")));
    logOptimised = new BooleanLogEntry(datalog, "SwerveModule/".concat(details.name().concat("/Optimisation")));
  }

  /** @return the module's drive wheel position (m) */
  public Distance getDrivePosition() {
    if (Robot.isSimulation())
      return simPose;

    return Meters.of(driveMotor.getPosition().getValue().in(Rotations) * DriveConstants.WHEEL_CIRCUMFERENCE.in(Meters));
  }

  /** @return the module's drive wheel velocity (m/s) */
  public LinearVelocity getDriveVelocity() {
    if (Robot.isSimulation())
      return simVelocity;

    return MetersPerSecond.of(
        driveMotor.getVelocity().getValue().in(RotationsPerSecond) *
            DriveConstants.WHEEL_CIRCUMFERENCE.in(Meters));
  }

  /**
   * 
   * @param robotRelative angles returned rel to (True : the robot front CCW+,
   *                      False : The current module)
   * @return the angle of the module relative to either Module Frame or Robot
   *         Frame
   */
  public Angle getTurnAngle() {
    double angle;
    if (Robot.isSimulation())
      angle = simAngle.in(Radians);
    else
      angle = turnEncoder.getPosition();

    return Radians.of(angle)
        .minus(Radians.of(details.angularOffset().getRadians()));
  }

  /** @return the module's turning angle as a {@link Rotation2d} */
  public Rotation2d getTurnRotation2d() {
    return new Rotation2d(getTurnAngle());
  }

  /** @return the module's turning velocity (rad/s) */
  public AngularVelocity getTurnVelocity() {
    if ( Robot.isSimulation() ){
      return simAngularRate;
    }
    return RadiansPerSecond.of(turnEncoder.getVelocity());
  }

  /** @return the module's current robot-relative position */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getDrivePosition(), getTurnRotation2d());
  }

  /** @return the module's current robot-relative state */
  public OptimisedSwerveModuleState getState() {
    return new OptimisedSwerveModuleState(getDriveVelocity(), getTurnAngle());
  }

  /** @return the module's current optimization status */
  public boolean getIsOptimized() {
    return desiredState.isOptimized();
  }

  /**
   * @return the desired state of the module, relative to the
   *         robot.
   */
  public OptimisedSwerveModuleState getDesiredState() {
    // we must convert to a robot-relative angle, since desiredState is relative to
    // the module.
    return new OptimisedSwerveModuleState(
        desiredState.speedMetersPerSecond,
        desiredState.angle);
  }

  /**
   * sets the desired state of the module.
   * 
   * @param state the desired state, relative to the robot.
   */
  public void setDesiredState(SwerveModuleState state) {
    setDesiredState(new OptimisedSwerveModuleState(state));
  }

  /**
   * sets the desired state of the module.
   * 
   * @param state the desired state, relative to the robot.
   */
  public void setDesiredState(OptimisedSwerveModuleState state) {
    // if the desired state's speed is low enough, and we are close enough to the
    // target angle we can just stop the motors to prevent motor weirdness
    if (Math.abs(state.speedMetersPerSecond) < 0.001) {
      state.speedMetersPerSecond = 0;
      if (Math.abs((state.angle.minus(getTurnRotation2d()).getRadians()) % 2 * Math.PI) < Math.PI / 16) { // ~11 degrees
        desiredState = state;
        stop();
        return;
      }
    }

    state.optimize(
        getTurnAngle(), getTurnVelocity(),
        RadiansPerSecond.of(3), RadiansPerSecondPerSecond.of(24), // TODO wrong constants
        desiredState);
    desiredState = state;
    applyState();
  }

  /** resets the drive encoder */
  public void resetEncoders() {
    driveMotor.setPosition(0);
  }

  /** stops both the drive and turning motors. */
  public void stop() {
    driveMotor.set(0);
    turnMotor.set(0);
  }

  /** apply current desired state to drive motors */
  private void applyState() {
    driveMotor.setControl(
        driveController.withVelocity(
            desiredState.speedMetersPerSecond / DriveConstants.WHEEL_CIRCUMFERENCE.in(Meters)));
    turnController.setReference(
        desiredState.angle.plus(details.angularOffset()).getRadians(),
        ControlType.kPosition);
  }

  public void runSim(){
    double degreesStep = 360;
    Angle targetAngle = Radians.of(desiredState.angle.plus(details.angularOffset()).getRadians());
    double angleDiff = targetAngle.minus(simAngle).in(Radians);
    angleDiff = angleDiff % (2 * Math.PI);
    angleDiff = (angleDiff < -Math.PI ? 2 * Math.PI + angleDiff : (angleDiff > Math.PI ? -2 * Math.PI + angleDiff : angleDiff));

    simAngularRate = angleDiff > 0 ? DegreesPerSecond.of(degreesStep) : DegreesPerSecond.of(-degreesStep);
    if (Math.abs(angleDiff) <= Math.abs(simAngularRate.in(RadiansPerSecond)/50)){
      simAngle = Radians.of(desiredState.angle.plus(details.angularOffset()).getRadians());
    } else {
      simAngle = simAngle.plus(Radians.of(simAngularRate.in(RadiansPerSecond)/50));
    }
    simVelocity = desiredState.getSpeed();
    simPose = simPose.plus(Meters.of(simVelocity.in(MetersPerSecond)/50));
  }

  /** @return a command that tests several motions of the swerve module */
  public SequentialCommandGroup testFunction() {
    return new SequentialCommandGroup(
        // turn to 0 degrees and check
        new InstantCommand(
            () -> setDesiredState(new OptimisedSwerveModuleState(MetersPerSecond.of(0), Rotation2d.kZero))),
        new WaitCommand(1),
        new ConditionalCommand(
            Subsystems.led.runPattern(LEDPattern.solid(Color.kGreen)),
            Subsystems.led.runPattern(LEDPattern.solid(Color.kRed)),
            () -> MathUtil.isNear((getTurnRotation2d().minus(Rotation2d.kZero)).getDegrees(), 0, 5))
            .withTimeout(0.5),

        // turn to 90 and check
        new InstantCommand(
            () -> setDesiredState(new OptimisedSwerveModuleState(MetersPerSecond.of(0), Rotation2d.kCCW_90deg))),
        new WaitCommand(1),
        new ConditionalCommand(
            Subsystems.led.runPattern(LEDPattern.solid(Color.kGreen)),
            Subsystems.led.runPattern(LEDPattern.solid(Color.kRed)),
            () -> MathUtil.isNear((getTurnRotation2d().minus(Rotation2d.kCCW_90deg)).getDegrees(), 0, 5))
            .withTimeout(0.5),

        // drive forwards and check
        new InstantCommand(
            () -> setDesiredState(new OptimisedSwerveModuleState(MetersPerSecond.of(1), Rotation2d.kCCW_90deg))),
        new WaitCommand(0.7),
        new ConditionalCommand(
            Subsystems.led.runPattern(LEDPattern.solid(Color.kGreen)),
            Subsystems.led.runPattern(LEDPattern.solid(Color.kRed)),
            () -> MathUtil.isNear(getDriveVelocity().in(MetersPerSecond) - 1, 0, 5))
            .withTimeout(0.5),

        // drive backwards and check
        new InstantCommand(
            () -> setDesiredState(new OptimisedSwerveModuleState(MetersPerSecond.of(-1), Rotation2d.kCCW_90deg))),
        new WaitCommand(1),
        new ConditionalCommand(
            Subsystems.led.runPattern(LEDPattern.solid(Color.kGreen)),
            Subsystems.led.runPattern(LEDPattern.solid(Color.kRed)),
            () -> MathUtil.isNear(getDriveVelocity().in(MetersPerSecond) - 1, 0, 5))
            .withTimeout(0.5),

        new InstantCommand(
            () -> setDesiredState(new OptimisedSwerveModuleState(MetersPerSecond.of(0), Rotation2d.kZero)))

    );
  }

  public void log(){
    logVTarg.append(desiredState.getSpeed().in(MetersPerSecond));
    logVTrue.append(getDriveVelocity().in(MetersPerSecond));
    logATarg.append(desiredState.getAngle().in(Radians));
    logATrue.append(getTurnAngle().in(Radians));
    logOptimised.append(desiredState.isOptimized());
  }
}