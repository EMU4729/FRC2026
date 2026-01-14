package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.opencv.core.Mat;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.IntakeConstants;

public class IntakeSub extends SubsystemBase {
  /* Init motor vars */
  private final TalonFX motor1 = new TalonFX(IntakeConstants.INTAKE_MOTOR_1_CANID);
  private final VelocityVoltage feederController1;

  private final double wheelRadius = 1.65/100; // m
  private final double ratio = 5; // input | output

  /* Init Motor Sim Vars */
  private final TalonFXSimState motor1Sim;
  private double simSpeed = 0;
  private double simSpeedTarget = 0;
  private final double simAccel = 0.5;



  public IntakeSub() {
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    /* Setup MotorConfig */
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.Feedback.SensorToMechanismRatio = 1;
    /* Untested PID values */
    motorConfig.Slot0.kP = 0;
    motorConfig.Slot0.kI = 0;
    motorConfig.Slot0.kD = 0;
    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    motor1.getConfigurator().apply(motorConfig);


    feederController1 = new VelocityVoltage(0).withSlot(0);


    motor1Sim = motor1.getSimState();

    
  }

  public void setSpeed(LinearVelocity speed){
    //AngularVelocity aSpeed = RotationsPerSecond.of(
    //  speed.in(MetersPerSecond) * ratio / (2*Math.PI*wheelRadius));
    
    AngularVelocity aSpeed = RadiansPerSecond.of(
      speed.in(MetersPerSecond) * ratio / wheelRadius);
    
    motor1.setControl(feederController1.withVelocity(aSpeed));
    
    if (Robot.isSimulation()) {
      simSpeedTarget = aSpeed.in(RadiansPerSecond);
    }
  }

  public LinearVelocity getSpeed() {
    AngularVelocity aSpeed = motor1.getVelocity().getValue(); // in rotations per second
    return MetersPerSecond.of(aSpeed.in(RotationsPerSecond) / ratio * (2*Math.PI*wheelRadius));
  }

  @Override 
  public void periodic() {
    LinearVelocity speeds = getSpeed();
    SmartDashboard.putNumber("Intake/motorSpeed", speeds.in(MetersPerSecond));
  }

  @Override
  public void simulationPeriodic() { 
    if (simSpeed > simSpeedTarget) {
      simSpeed -= simAccel;
    } else if (simSpeed < simSpeedTarget) {
      simSpeed += simAccel;
    }

    motor1Sim.setRotorVelocity(simSpeed);
  
  }
}