package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.TurretConstants;
import frc.robot.constants.TurretFeederConstants;

public class TurretShooterSub extends SubsystemBase{
    private final TalonFX motor1 = new TalonFX(TurretConstants.TURRENT_MOTOR_1_CANID);
    private final TalonFX motor2 = new TalonFX(TurretConstants.TURRENT_MOTOR_2_CANID);
    private final VelocityVoltage feederController1 = new VelocityVoltage(0).withSlot(0);
    //private final VelocityVoltage feederController2 = new VelocityVoltage(0).withSlot(0);
    

    private final Distance wheelRadius = Meters.of(Units.inchesToMeters(4)); // meters, unknown at the moment

    //sim
    private final double ratio = 1; 

    private final TalonFXSimState motor1Sim;
    private final TalonFXSimState motor2Sim;
    private double simSpeed = 0;
    private double simSpeedTarget = 0;
    private final double simAccel = 0.5;
    
    public TurretShooterSub(){
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfig.Feedback.SensorToMechanismRatio = 1;
        //unknown PID
        motorConfig.Slot0.kP = 1;
        motorConfig.Slot0.kI = 0;
        motorConfig.Slot0.kD = 0;
        motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        motor1.getConfigurator().apply(motorConfig);

        
        motor1Sim = motor1.getSimState();
        motor2Sim = motor2.getSimState();
        motor2.setControl(new Follower(motor1.getDeviceID(), MotorAlignmentValue.Aligned));
    }
    

     public void setSpeed(LinearVelocity speed) {

        AngularVelocity aSpeed = RadiansPerSecond.of(
            speed.in(MetersPerSecond) * ratio /wheelRadius.in(Meters));

            motor1.setControl(feederController1.withVelocity(aSpeed));
            //motor2.setControl(feederController2.withVelocity(aSpeed));

            if (Robot.isSimulation()) {
                simSpeedTarget = aSpeed.in(RadiansPerSecond);
            }
    }


    public LinearVelocity getSpeed() {
        AngularVelocity aSpeed = motor1.getVelocity().getValue(); // in rotations per second
        return MetersPerSecond.of(aSpeed.in(RotationsPerSecond) / ratio * (2*Math.PI*wheelRadius.in(Meters)));
    }

    public boolean atspeed(){
        if (getSpeed().in(MetersPerSecond) >= TurretConstants.MIN_MOTOR_SPEED){
            return true;
        } else {
            return false;
        }
    }

     @Override 
    public void periodic() {
        LinearVelocity speeds = getSpeed();
        SmartDashboard.putNumber("Turret/motorSpeed", speeds.in(MetersPerSecond));
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
  
  }
}
