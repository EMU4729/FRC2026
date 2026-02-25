package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Subsystems;
import frc.robot.constants.AimingConstants;
import frc.robot.constants.TurretConstants;

public class TurretAimingSub extends SubsystemBase {   
    //Hardwares
    private final TalonFX slewMotor = new TalonFX(AimingConstants.SLEW_MOTOR_CANID);
    private final TalonFX hoodMotor = new TalonFX(AimingConstants.HOOD_MOTOR_CANID); //TODO CHANGE THIS

    private final PositionVoltage slewController = new PositionVoltage(0).withSlot(0);
    private final PositionVoltage hoodController = new PositionVoltage(0).withSlot(0);

    private Angle manualSlewTarget = Degrees.of(0);
    private Angle manualHoodTarget = Degrees.of(0);

    //Simulation  
    private final TalonFXSimState slewMotorSim;
    private final TalonFXSimState hoodMotorSim;
    

    public TurretAimingSub() {
       // --- Turret Configuration ---
        TalonFXConfiguration aimingConfig = new TalonFXConfiguration();
        aimingConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        aimingConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.5;
        aimingConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        aimingConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -0.5;
        aimingConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        aimingConfig.Feedback.SensorToMechanismRatio = TurretConstants.rotatorMotorRatio; // DriveConstants.DRIVE_GEAR_RATIO;
        aimingConfig.Slot0.kP = TurretConstants.ROTATOR_P;
        aimingConfig.Slot0.kI = TurretConstants.ROTATOR_I;
        aimingConfig.Slot0.kD = TurretConstants.ROTATOR_D;
        aimingConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        slewMotor.getConfigurator().apply(aimingConfig);
 

        TalonFXConfiguration hoodMotorConfig;
        hoodMotorConfig = new TalonFXConfiguration();
        hoodMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        hoodMotorConfig.Feedback.SensorToMechanismRatio = TurretConstants.hoodMotorRatio;
        hoodMotorConfig.Slot0.kP = TurretConstants.HOOD_P;
        hoodMotorConfig.Slot0.kI = TurretConstants.HOOD_I;
        hoodMotorConfig.Slot0.kD = TurretConstants.HOOD_D;
        hoodMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        hoodMotor.getConfigurator().apply(hoodMotorConfig);


        slewMotorSim = slewMotor.getSimState();
        hoodMotorSim = hoodMotor.getSimState();

        setupSmartDash();
    }

    @Override
    public void periodic() {        
        Angle calculatedTargetRot;
        Angle calculatedTargetHood;

        // 1. Determine Input Targets
        calculatedTargetRot = manualSlewTarget;
            
        calculatedTargetHood = manualHoodTarget;

        slewMotor.setControl(
            slewController
                .withPosition(calculatedTargetRot));

        hoodMotor.setControl(
            hoodController
                .withPosition(calculatedTargetHood));

        updateTelemetry(calculatedTargetRot);
    }

    public Angle getTurretAngle() {
        return slewMotor.getPosition().getValue();
    }

    public Angle getHoodAngle() {
        return hoodMotor.getPosition().getValue();
    }

    /** sets the field relative yaw the turret should aim at */
    public void setSlewTarget(Angle targetSlewAngle){
        this.manualSlewTarget = targetSlewAngle;
    }

    public void setHoodTarget(Angle targetHoodAngle){
        this.manualHoodTarget = targetHoodAngle;
    
        hoodMotor.setControl(new PositionVoltage(targetHoodAngle.div(360)));
    }

    public void stop(){
        //TODO
        slewMotor.stopMotor();
        hoodMotor.stopMotor();
        // stops all movement
    }

   

    private void updateTelemetry(Angle targetRot) {
        SmartDashboard.putNumber("Turret/Degrees", getTurretAngle().in(Degrees));
        SmartDashboard.putNumber("Turret/TargetDegrees", manualSlewTarget.in(Degrees));
        SmartDashboard.putNumber("Hood/Degrees", getHoodAngle().in(Degrees));
        SmartDashboard.putNumber("Hood/TargetDegrees", manualHoodTarget.in(Degrees));

        // Draw a "Beam" on Field2d
        Pose2d turretPointer = new Pose2d(
                AimingConstants.robotToTurret.getTranslation(),
                Rotation2d.fromRadians(-getTurretAngle().in(Radians))
            );
        Subsystems.nav.drawFieldObject("Turret", turretPointer, true);
    }

    public void setupSmartDash() {
        SmartDashboard.putData("Turret Sub", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("Turret");
                // Corrected names and use getDegrees() for readability
                builder.addDoubleProperty("Rotator Angle (deg)", () -> getTurretAngle().in(Degrees), null);
                builder.addDoubleProperty("Hood Angle (deg)", () -> getHoodAngle().in(Degrees), null);
                builder.addDoubleProperty("Rotator Velocity", () -> slewMotor.getVelocity().getValueAsDouble(), null);
            }
        });
    }

    
  @Override
  public void simulationPeriodic() {
    // TODO: handle simulation (doesn't have whole module support like the swerve module)
    slewMotorSim.setRawRotorPosition(manualSlewTarget.in(Rotations) / TurretConstants.rotatorMotorRatio);
    hoodMotorSim.setRawRotorPosition(manualHoodTarget.in(Rotations) / TurretConstants.hoodMotorRatio);
  }
}
