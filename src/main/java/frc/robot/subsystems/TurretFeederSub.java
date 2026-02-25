package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Radians;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.TurretFeederConstants;

public class TurretFeederSub extends SubsystemBase{
    /* Initial motor variables */
    private final TalonFX motor1 = new TalonFX(TurretFeederConstants.TURRENT_MOTOR_1_CANID);
    private final TalonFX motor2 = new TalonFX(TurretFeederConstants.TURRENT_MOTOR_1_CANID);
   
    private final VelocityVoltage feederController1 = new VelocityVoltage(0).withSlot(0);;
   private final PositionVoltage positionControl = new PositionVoltage(0);

    private final Distance wheelRadius = Meters.of(1/100); // meters, unknown at the moment

    //sim
    private final double ratio = 1; 

    private final TalonFXSimState motor1Sim;
    private double simSpeed = 0;
    private double simSpeedTarget = 0;
    private final double simAccel = 0.5;

    public TurretFeederSub(){
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfig.Feedback.SensorToMechanismRatio = 1;
        //unknown PID
        motorConfig.Slot0.kP = 1;
        motorConfig.Slot0.kI = 0;
        motorConfig.Slot0.kD = 0;
        motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        motor1.getConfigurator().apply(motorConfig);
        motor2.setControl(new Follower(0, null));
        
        motor1Sim = motor1.getSimState();
      
    }


    public void setSpeed(LinearVelocity speed) {

        AngularVelocity aSpeed = RadiansPerSecond.of(
            speed.in(MetersPerSecond) * ratio /wheelRadius.in(Meters));

            motor1.setControl(feederController1.withVelocity(aSpeed));

            if (Robot.isSimulation()) {
                simSpeedTarget = aSpeed.in(RadiansPerSecond);
            }
    }
    public void setTargetAngle(Rotation2d angle) {
       double gearRatio = 1.0; //TODO CHANGE THIS LATER
    motor1.setControl(positionControl.withPosition(angle.getRotations() * gearRatio));
    }
    public void stop() {
        motor1.stopMotor();
        motor2.stopMotor();
    }
    public void setSpeedFromAngular(AngularVelocity speed) {
    // motor1.setControl(feederController1.withVelocity(speed));
    // If using CTRE VelocityVoltage, it usually expects Rotations per Second
    motor1.setControl(feederController1.withVelocity(speed.in(RotationsPerSecond)));

    if (Robot.isSimulation()) {
        simSpeedTarget = speed.in(RadiansPerSecond);
    }
}


    public LinearVelocity getSpeed() {
        AngularVelocity aSpeed = motor1.getVelocity().getValue(); // in rotations per second
        return MetersPerSecond.of(aSpeed.in(RotationsPerSecond) / ratio * (2*Math.PI*wheelRadius.in(Meters)));
    }

     @Override 
    public void periodic() {
        LinearVelocity speeds = getSpeed();
        SmartDashboard.putNumber("TurretFeeder/motorSpeed", speeds.in(MetersPerSecond));
    }

    
      @Override
    public void simulationPeriodic() { 
         if (simSpeed > simSpeedTarget) {
        simSpeed -= simAccel;
    }   else if (simSpeed < simSpeedTarget) {
         simSpeed += simAccel;
    }

        motor1Sim.setRotorVelocity(simSpeed);
  
  }


}

