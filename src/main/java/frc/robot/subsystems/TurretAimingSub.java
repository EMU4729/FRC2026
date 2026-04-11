package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
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
import frc.robot.constants.DriveConstants;
import frc.robot.constants.TurretConstants;

public class TurretAimingSub extends SubsystemBase {
    // Hardwares
    private final TalonFX slewMotor = new TalonFX(AimingConstants.SLEW_MOTOR_CANID);
    private final TalonFX hoodMotor_1 = new TalonFX(AimingConstants.HOOD_MOTOR_CANID_1); // TODO CHANGE THIS
    private final TalonFX hoodMotor_2 = new TalonFX(AimingConstants.HOOD_MOTOR_CANID_2); // TODO CHANGE THIS

    private final PositionVoltage slewController = new PositionVoltage(0).withSlot(0);
    private final PositionVoltage hoodController_1 = new PositionVoltage(0).withSlot(0);
    private final PositionVoltage hoodController_2 = new PositionVoltage(0).withSlot(0);

    private Angle manualSlewTarget = Degrees.of(0);
    private Angle manualHoodTarget = Degrees.of(0);

    // Simulation
    private final TalonFXSimState slewMotorSim;
    private final TalonFXSimState hoodMotorSim;

    public TurretAimingSub() {
        // --- Turret Configuration ---
        TalonFXConfiguration aimingConfig = new TalonFXConfiguration();
        aimingConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        aimingConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.25;
        aimingConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        aimingConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -0.25;
        aimingConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        aimingConfig.Feedback.SensorToMechanismRatio = TurretConstants.rotatorMotorRatio; // DriveConstants.DRIVE_GEAR_RATIO;
        aimingConfig.Slot0.kP = TurretConstants.ROTATOR_P;
        aimingConfig.Slot0.kI = TurretConstants.ROTATOR_I;
        aimingConfig.Slot0.kD = TurretConstants.ROTATOR_D;
        aimingConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        //slewMotor.getConfigurator().apply(aimingConfig);

        TalonFXConfiguration hoodMotorConfig_1;
        hoodMotorConfig_1 = new TalonFXConfiguration();
        hoodMotorConfig_1.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        hoodMotorConfig_1.Feedback.SensorToMechanismRatio = TurretConstants.hoodMotorRatio;
        hoodMotorConfig_1.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        hoodMotorConfig_1.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.25;
        hoodMotorConfig_1.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        hoodMotorConfig_1.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
        hoodMotorConfig_1.Slot0.kP = TurretConstants.HOOD_P;
        hoodMotorConfig_1.Slot0.kI = TurretConstants.HOOD_I;
        hoodMotorConfig_1.Slot0.kD = TurretConstants.HOOD_D;
        hoodMotorConfig_1.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        hoodMotor_1.getConfigurator().apply(hoodMotorConfig_1);

        TalonFXConfiguration hoodMotorConfig_2;
        hoodMotorConfig_2 = new TalonFXConfiguration();
        hoodMotorConfig_2.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        hoodMotorConfig_2.Feedback.SensorToMechanismRatio = TurretConstants.hoodMotorRatio;
        hoodMotorConfig_2.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        hoodMotorConfig_2.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.25;
        hoodMotorConfig_2.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        hoodMotorConfig_2.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
        hoodMotorConfig_2.Slot0.kP = TurretConstants.HOOD_P;
        hoodMotorConfig_2.Slot0.kI = TurretConstants.HOOD_I;
        hoodMotorConfig_2.Slot0.kD = TurretConstants.HOOD_D;
        hoodMotorConfig_2.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        hoodMotor_2.getConfigurator().apply(hoodMotorConfig_2);

        slewMotorSim = slewMotor.getSimState();
        hoodMotorSim = hoodMotor_1.getSimState();

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

        hoodMotor_1.setControl(
                hoodController_1
                        .withPosition(calculatedTargetHood));

        hoodMotor_2.setControl(
                hoodController_2
                        .withPosition(calculatedTargetHood));

        updateTelemetry(calculatedTargetRot);
    }

    public Angle getTurretAngle() {
        return slewMotor.getPosition().getValue();
    }

    public Angle getHoodAngle() {
        return hoodMotor_1.getPosition().getValue();
    }

    /** sets the field relative yaw the turret should aim at */
    public void setSlewTarget(Angle targetSlewAngle) {
        this.manualSlewTarget = targetSlewAngle;
    }

    public void setHoodTarget(Angle targetHoodAngle) {
        this.manualHoodTarget = targetHoodAngle;
        
        hoodMotor_1.setControl(hoodController_1.withPosition(targetHoodAngle));
        hoodMotor_2.setControl(hoodController_2.withPosition(targetHoodAngle));
    }
    
    public void stop() {
        this.manualHoodTarget = Degrees.of(0);
        // TODO
        slewMotor.stopMotor();
        hoodMotor_1.setControl(hoodController_1.withPosition(Degrees.of(0)));
        hoodMotor_2.setControl(hoodController_2.withPosition(Degrees.of(0)));
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
                Rotation2d.fromRadians(-getTurretAngle().in(Radians)));
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
                builder.addDoubleProperty("Hood Angle target", () -> slewMotor.getVelocity().getValueAsDouble(), null);
            }
        });
    }

    @Override
    public void simulationPeriodic() {
        // TODO: handle simulation (doesn't have whole module support like the swerve
        // module)
        slewMotorSim.setRawRotorPosition(manualSlewTarget.in(Rotations) / TurretConstants.rotatorMotorRatio);
        hoodMotorSim.setRawRotorPosition(manualHoodTarget.in(Rotations) / TurretConstants.hoodMotorRatio);
    }
}
