package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.List;
import java.util.Optional;

import static edu.wpi.first.units.Units.Radians;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Subsystems;
import frc.robot.constants.AimingConstants;
import frc.robot.constants.AimingConstants.DistanceSample;
import frc.robot.constants.AimingConstants.TurretState;
import frc.robot.constants.TurretConstants;
import frc.robot.constants.TurretFeederConstants;
import frc.robot.utils.TurretAiming;

public class TurretAimingSub extends SubsystemBase {
   //PIDs
   
    //Samples
    private final List<DistanceSample> aimingTable = AimingConstants.PassingSamples;
   
    //Hardwares
    private final TalonFX aimingMotor = new TalonFX(TurretFeederConstants.TURRENT_MOTOR_1_CANID);
    private final TalonFX HoodMotor = new TalonFX(TurretFeederConstants.TURRENT_MOTOR_1_CANID); //TODO CHANGE THIS
    private TurretState currentTargetState;
    private Rotation2d manualSlewTarget = null;
    private Angle manualHoodTarget = null;

    //Simulation  
    private final TalonFXSimState rotatorMotorSim;
    private final TalonFXSimState hoodMotorSim;
    
 private final MotionMagicVoltage aimingController = new MotionMagicVoltage(0);
private final MotionMagicVoltage hoodController = new MotionMagicVoltage(0);

   private final TalonFXConfiguration hoodMotorConfig;

    public TurretAimingSub() {
       // --- Turret Configuration ---
        TalonFXConfiguration aimingConfig = new TalonFXConfiguration();
        aimingConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    aimingConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.5;
    aimingConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    aimingConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -0.5;
    aimingConfig.MotionMagic.MotionMagicCruiseVelocity = 10; // Rotations per second
    aimingConfig.MotionMagic.MotionMagicAcceleration = 20;   // Rotations per second^2
    aimingConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    aimingConfig.Feedback.SensorToMechanismRatio = TurretConstants.rotatorMotorRatio; // DriveConstants.DRIVE_GEAR_RATIO;
    aimingConfig.Slot0.kP = TurretConstants.ROTATOR_P;
    aimingConfig.Slot0.kI = TurretConstants.ROTATOR_I;
    aimingConfig.Slot0.kD = TurretConstants.ROTATOR_D;
    aimingConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    // aimingConfig.SoftwareLimitSwitch.
    aimingMotor.getConfigurator().apply(aimingConfig);
 

  
    hoodMotorConfig = new TalonFXConfiguration();
    hoodMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    hoodMotorConfig.Feedback.SensorToMechanismRatio = TurretConstants.hoodMotorRatio;
    hoodMotorConfig.Slot0.kP = TurretConstants.HOOD_P;
    hoodMotorConfig.Slot0.kI = TurretConstants.HOOD_I;
    hoodMotorConfig.Slot0.kD = TurretConstants.HOOD_D;
    hoodMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 5;
    hoodMotorConfig.MotionMagic.MotionMagicAcceleration = 10;
    HoodMotor.getConfigurator().apply(hoodMotorConfig);


    rotatorMotorSim = aimingMotor.getSimState();
    hoodMotorSim = HoodMotor.getSimState();

    setupSmartDash();
    }

   @Override
    public void periodic() {
        Pose2d robotPose = Subsystems.nav.getPose();
        
        Rotation2d calculatedTargetRot;
        Angle calculatedTargetHood;

        // 1. Determine Input Targets
        if (manualSlewTarget != null || manualHoodTarget != null) {
            calculatedTargetRot = (manualSlewTarget != null) ? 
                manualSlewTarget.minus(robotPose.getRotation()) : getTurretAngle();
            
            calculatedTargetHood = (manualHoodTarget != null) ? 
                manualHoodTarget : getHoodAngle();
        } else {
            Optional<AprilTag> target = TurretAiming.getBestTarget(robotPose);
            if (target.isPresent()) {
                Translation2d targetTrans = target.get().pose.getTranslation().toTranslation2d();
                Translation2d relativeTarget = targetTrans.minus(robotPose.getTranslation());
                TurretState state = TurretAiming.calcState(aimingTable, relativeTarget);
                
                calculatedTargetRot = Rotation2d.fromDegrees(state.turretAngle().in(Degrees));
                calculatedTargetHood = state.hoodAngle();
                
                Subsystems.nav.drawFieldObject("TurretTarget", new Pose2d(targetTrans, new Rotation2d()), false);
            } else {
                calculatedTargetRot = getTurretAngle(); 
                calculatedTargetHood = getHoodAngle();
            }
        }

     calculatedTargetRot = Rotation2d. fromRotations(calculatedTargetRot.getRotations());


    // Use MotionMagic for smooth, profiled movement handled on-device
aimingMotor.setControl(aimingController.withPosition(calculatedTargetRot.getRotations()));
HoodMotor.setControl(hoodController.withPosition(calculatedTargetHood.in(Rotations)));
        // 4. Telemetry & Field Visualization
        updateTelemetry(calculatedTargetRot, calculatedTargetHood);
    }

    public Rotation2d getTurretAngle() {
      
        return Rotation2d.fromRotations(aimingMotor.getPosition().getValueAsDouble());
    }
    public Angle getHoodAngle() {
        return HoodMotor.getPosition().getValue();
    }

    public void setMotorSpeed(double speed) {
        aimingMotor.set(speed);
    }


    public void setSlewTarget(Angle targetSlewAngle){
        //TODO
            this.manualSlewTarget = Rotation2d.fromDegrees(targetSlewAngle.in(Degrees));

        // sets the field relative yaw the turret should aim at
    }

    public void setHoodTarget(Angle targetHoodAngle){
    
    }

    public void stop(){
        //TODO
        aimingMotor.stopMotor();
        HoodMotor.stopMotor();
        // stops all movement
    }

   

     private void updateTelemetry(Rotation2d targetRot, Angle targetHood) {
        SmartDashboard.putNumber("Turret/Degrees", getTurretAngle().getDegrees());
        SmartDashboard.putNumber("Turret/TargetDegrees", targetRot.getDegrees());
        SmartDashboard.putNumber("Hood/Degrees", getHoodAngle().in(Degrees));
        SmartDashboard.putNumber("Hood/TargetDegrees", targetHood.in(Degrees));

        // Draw a "Beam" on Field2d
        Rotation2d fieldRelativeTurretAngle = getTurretAngle().plus(Subsystems.nav.getPose().getRotation());
        Pose2d turretPointer = new Pose2d(
            Subsystems.nav.getPose().getTranslation().plus(new Translation2d(2.0, fieldRelativeTurretAngle)),
            fieldRelativeTurretAngle
        );
        Subsystems.nav.drawFieldObject("TurretBeam", turretPointer, false);
    }

    public void setupSmartDash() {
        SmartDashboard.putData("Turret Sub", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("Turret");
                // Corrected names and use getDegrees() for readability
                builder.addDoubleProperty("Rotator Angle (deg)", () -> getTurretAngle().getDegrees(), null);
                builder.addDoubleProperty("Hood Angle (deg)", () -> getHoodAngle().in(Degrees), null);
                builder.addDoubleProperty("Rotator Velocity", () -> aimingMotor.getVelocity().getValueAsDouble(), null);
            }
        });
    }
}
