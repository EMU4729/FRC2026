package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.constants.IntakeConstants;

public class IntakeSub extends SubsystemBase {
  /* Init motor vars */
  private final TalonFX intakeMotor = new TalonFX(IntakeConstants.INTAKE_MOTOR_1_CANID);
  private final VelocityVoltage feederController1;

  private final TalonFX pivotMotor = new TalonFX(IntakeConstants.INTAKE_DEPLOY_MOTOR_CANID);
  private final PositionVoltage feedercontroller2;

  private Angle extendTargetAngle = Degrees.of(0);

  private final double wheelRadius = 1.65 / 100; // m
  private final double ratio = 5; // input | output

  public double angleOffset = 0;

  /* Init Motor Sim Vars */
  private final TalonFXSimState motor1Sim;
  private double simSpeed = 0;
  private double simSpeedTarget = 0;
  private final double simAccel = 0.5;

  private final TalonFXSimState motor2Sim;
  private double simSpeed2 = 0;
  private double simSpeedTarget2 = 0;
  private final double simAccel2 = 0.5;

  private final SlewRateLimiter pivotPositionRateLimiter = new SlewRateLimiter(240);

  public IntakeSub() {
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    /* Setup MotorConfig */
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.Feedback.SensorToMechanismRatio = 1;
    /* Untested PID values */
    motorConfig.Slot0.kP = 1;
    motorConfig.Slot0.kI = 0.2;
    motorConfig.Slot0.kD = 0;
    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    intakeMotor.getConfigurator().apply(motorConfig);

    feederController1 = new VelocityVoltage(0).withSlot(0);

    motor1Sim = intakeMotor.getSimState();

    TalonFXConfiguration motorDeployConfig = new TalonFXConfiguration();
    /* Setup MotorConfig */
    motorDeployConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorDeployConfig.Feedback.SensorToMechanismRatio = 58;
    /* tested PID values */
    motorDeployConfig.Slot0.kP = 20;
    motorDeployConfig.Slot0.kI = 5;
    motorDeployConfig.Slot0.kD = 0;
    motorDeployConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    pivotMotor.getConfigurator().apply(motorDeployConfig);
    pivotMotor.setPosition(Degrees.of(0));
    feedercontroller2 = new PositionVoltage(0).withSlot(0);

    pivotPositionRateLimiter.calculate(0);
    motor2Sim = pivotMotor.getSimState();

    SmartDashboard.putNumber("Intake/Angle_Offset", angleOffset);

    setExtendAngle();
  }

  public void setSpeed(LinearVelocity speed) {
    if (speed.in(MetersPerSecond) == 0) {
      stop();
      return;
    }
    // AngularVelocity aSpeed = RotationsPerSecond.of(
    // speed.in(MetersPerSecond) * ratio / (2*Math.PI*wheelRadius));

    AngularVelocity aSpeed = RadiansPerSecond.of(
        speed.in(MetersPerSecond) * ratio / wheelRadius);

    intakeMotor.setControl(feederController1.withVelocity(aSpeed));

    if (Robot.isSimulation()) {
      simSpeedTarget = aSpeed.in(RadiansPerSecond);
    }
  }

  public void stop() {
    intakeMotor.setControl(new DutyCycleOut(0));
  }

  public LinearVelocity getSpeed() {
    AngularVelocity aSpeed = intakeMotor.getVelocity().getValue(); // in rotations per second
    return MetersPerSecond.of(aSpeed.in(RotationsPerSecond) / ratio * (2 * Math.PI * wheelRadius));
  }

  public void setExtendAngle() {
    extendTargetAngle = IntakeConstants.EXTEND_ANGLE;
  }

  public void setShootExtendAngle() {
    if (extendTargetAngle.lt(IntakeConstants.SHOOT_ANGLE)) {
      return;
    }
    extendTargetAngle = IntakeConstants.SHOOT_ANGLE;
  }

  public void setShootRetractAngle() {
    if (extendTargetAngle.gt(IntakeConstants.SHOOT_ANGLE)) {
      return;
    }
    extendTargetAngle = IntakeConstants.RETRACT_ANGLE;
  }

  public void setRetractedAngle() {
    extendTargetAngle = IntakeConstants.RETRACT_ANGLE;
  }

  @Override
  public void periodic() {
    LinearVelocity speeds = getSpeed();

    angleOffset = SmartDashboard.getNumber("Intake/Angle_Offset", angleOffset);
    SmartDashboard.putNumber("Intake/motorSpeed", speeds.in(MetersPerSecond));

    if (DriverStation.isEnabled()) {
      pivotMotor.setControl(
          feedercontroller2.withPosition(Degrees.of(
              pivotPositionRateLimiter.calculate(
                  extendTargetAngle.in(Degrees)
                      + angleOffset))));
    } else {
      pivotPositionRateLimiter.calculate(0);
    }

  }

  public Command adjustAngleOffsetCommand(int by) {
    return new SequentialCommandGroup(
        new InstantCommand(() -> {
          final var currentIntakeAngleOffset = SmartDashboard.getNumber("Intake/Angle_Offset", angleOffset);
          SmartDashboard.putNumber("Intake/Angle_Offset", currentIntakeAngleOffset + by);
        }),
        new WaitCommand(0.3)).repeatedly();
  }

  @Override
  public void simulationPeriodic() {
    if (simSpeed > simSpeedTarget) {
      simSpeed -= simAccel;
    } else if (simSpeed < simSpeedTarget) {
      simSpeed += simAccel;
    }

    motor1Sim.setRotorVelocity(simSpeed);
    motor2Sim.setRawRotorPosition(extendTargetAngle);

  }
}