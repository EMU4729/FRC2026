package frc.robot.commands.Turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;
import frc.robot.constants.AimingConstants;
import frc.robot.constants.TurretConstants;
import frc.robot.constants.AimingConstants.TurretState;
import frc.robot.constants.TurretFeederConstants;
import frc.robot.utils.TurretAiming;

public class TurretShootAtHub extends Command {

    public double testVel = 13;
    public double testAng = 45;

    /** Amps above which we consider a ball to be passing through the feeder. */
    private static final double FEEDER_LOADED_CURRENT_AMPS = 5.0;

    /**
     * If no current spike is detected for this long, we consider all balls shot.
     * Resets every time a ball is detected.
     */
    private static final double NO_BALL_TIMEOUT_SECONDS = 2.5;

    /** Minimum run time before the finish condition is armed. */
    private static final double MIN_RUN_SECONDS = 13;

    private Translation2d ourHub = AimingConstants.Red_Hub;

    private final Timer runTimer      = new Timer();
    private final Timer noBallTimer   = new Timer();

    // Flipped to true when noBallTimer expires — checked by .until() in forAuto()
    private boolean done = false;

    public TurretShootAtHub() {
        addRequirements(Subsystems.turretAiming, Subsystems.turretFeeder, Subsystems.turretShooter);
        SmartDashboard.putNumber("Turret/ShooterPow", testVel);
        SmartDashboard.putNumber("Turret/ShooterAng", testAng);
    }

    @Override
    public void initialize() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        ourHub = (alliance.isPresent() && alliance.get() == Alliance.Red)
                ? AimingConstants.Red_Hub
                : AimingConstants.Blue_Hub;

        Subsystems.nav.drawFieldObject("TurretTarget", new Pose2d(ourHub, new Rotation2d()), false);

        runTimer.restart();
        noBallTimer.restart();
        done = false;
    }

    @Override
    public void execute() {

        // Rotate robot to face the hub target instead of rotating the turret
        // Subsystems.drive.driveAtAngle(new ChassisSpeeds(0, 0, 0), true,
        //         Rotation2d.fromRadians(HubCalc.turretAngle().in(Radians)));

        TurretState hubCalc = TurretAiming.calcState(AimingConstants.ShootingSamples, ourHub);
        
        testAng = SmartDashboard.getNumber("Turret/ShooterAng", 0);
        testVel = SmartDashboard.getNumber("Turret/ShooterPow", 0);
        Subsystems.turretAiming.setHoodTarget(Degrees.of(testAng));
        Subsystems.turretShooter.setSpeed(MetersPerSecond.of(testVel));    
        //Subsystems.turretAiming.setHoodTarget(hubCalc.hoodAngle());
        //Subsystems.turretShooter.setSpeed(hubCalc.power());
        Subsystems.turretFeeder.popFuel(TurretFeederConstants.TARGET_SPEED);


        // ── ball detection via current spike ────────────────────────
        boolean ballDetected = Subsystems.turretFeeder.getSupplyCurrent()
                               >= FEEDER_LOADED_CURRENT_AMPS;

        if (ballDetected) {
            // Ball passing through — reset the silence window
            noBallTimer.restart();
        }

        // Arm the done flag only after minimum run time has elapsed
        if (runTimer.hasElapsed(MIN_RUN_SECONDS)
                && noBallTimer.hasElapsed(NO_BALL_TIMEOUT_SECONDS)) {
            done = true;
        }
    }

   

    @Override
    public void end(boolean interrupted) {
        runTimer.stop();
        noBallTimer.stop();

        Subsystems.turretAiming.stop();
        Subsystems.turretShooter.setSpeed(TurretConstants.ShooterIdleSpeed);
        Subsystems.turretFeeder.stop();
    }

    /**
     * Auto factory — runs until 2.5s of silence after the last detected ball,
     * using .until() so isFinished() stays clean.
     */
    public static Command forAuto() {
        TurretShootAtHub cmd = new TurretShootAtHub();
        return cmd.until(() -> cmd.done);
    }
}