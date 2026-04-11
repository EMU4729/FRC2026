package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.TurretConstants;

public class TurretShooterSub extends SubsystemBase{
    private final TalonFX motor1 = new TalonFX(TurretConstants.SHOOTER_MOTOR_1_CANID);
    private final TalonFX motor2 = new TalonFX(TurretConstants.SHOOTER_MOTOR_2_CANID);
    private final TalonFX motor3 = new TalonFX(TurretConstants.SHOOTER_MOTOR_3_CANID);
    private final VelocityVoltage feederController1 = new VelocityVoltage(0).withSlot(0);
    //private final VelocityVoltage feederController2 = new VelocityVoltage(0).withSlot(0);

    private Timer atSpeedStayOn = new Timer();
    

    private final Distance wheelRadius = Inches.of(2); // meters, unknown at the moment

    //sim
    private final double ratio = 1; 

    private final TalonFXSimState motor1Sim;
    private final TalonFXSimState motor2Sim;
    private final TalonFXSimState motor3Sim;
    private double simSpeed = 0;
    private double simSpeedTarget = 0;
    private final double simAccel = 0.5;
    private LinearVelocity targetSpeed = MetersPerSecond.of(0);
    
    public TurretShooterSub(){
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        motorConfig.Feedback.SensorToMechanismRatio = 1;
        //unknown PID
        motorConfig.Slot0.kP = 0.3;
        motorConfig.Slot0.kI = 1.2;
        motorConfig.Slot0.kD = 0;
        motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        motor1.getConfigurator().apply(motorConfig);
        motor2.getConfigurator().apply(motorConfig);
        motor3.getConfigurator().apply(motorConfig);

        
        motor1Sim = motor1.getSimState();
        motor2Sim = motor2.getSimState();
        motor3Sim = motor3.getSimState();
        motor2.setControl(new Follower(motor1.getDeviceID(), MotorAlignmentValue.Opposed));
        motor3.setControl(new Follower(motor1.getDeviceID(), MotorAlignmentValue.Opposed));
        
        atSpeedStayOn.start();

        stop();
    }
    

    public void setSpeed(LinearVelocity speed) {

        AngularVelocity aSpeed = RadiansPerSecond.of(
                speed.in(MetersPerSecond) * ratio /wheelRadius.in(Meters));

        motor1.setControl(feederController1.withVelocity(aSpeed));

        //motor2.setControl(feederController2.withVelocity(aSpeed));
        targetSpeed = speed;
        if (Robot.isSimulation()) {
            simSpeedTarget = aSpeed.in(RadiansPerSecond);
        }
    }

    public void stop(){
        //setSpeed(TurretConstants.ShooterIdleSpeed);
        motor1.setControl(new DutyCycleOut(0));
    }

    public LinearVelocity getSpeed() {
        AngularVelocity aSpeed = motor1.getVelocity().getValue(); // in rotations per second
        return MetersPerSecond.of(aSpeed.in(RotationsPerSecond) / ratio * (2*Math.PI*wheelRadius.in(Meters)));
    }

    public boolean atspeed(){
        if (targetSpeed == null) return false;
        if (targetSpeed.minus(getSpeed()).in(MetersPerSecond) < 0.5 && 
                targetSpeed.gt(MetersPerSecond.of(6))) {
            atSpeedStayOn.reset();
            return true;
        } else {

            return !atSpeedStayOn.hasElapsed(0.5);
        }
    }

    @Override 
    public void periodic() {
        LinearVelocity speeds = getSpeed();
        SmartDashboard.putNumber("Turret/motorSpeed", speeds.in(MetersPerSecond));
        SmartDashboard.putNumber("Turret/motorTargSpeed", targetSpeed.in(MetersPerSecond));
    }

    
    @Override
    public void simulationPeriodic() { 
        if (simSpeed > simSpeedTarget) {
            simSpeed -= simAccel;
        }   else if (simSpeed < simSpeedTarget) {
            simSpeed += simAccel;
        }

        motor1Sim.setRotorVelocity(simSpeed);
        motor2Sim.setRotorVelocity(simSpeed);
        motor3Sim.setRotorVelocity(simSpeed);
  
    }
}
