package frc.robot.commands.Turret;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;
import frc.robot.constants.AimingConstants;
import frc.robot.constants.AimingConstants.TurretState;
import frc.robot.constants.TurretFeederConstants;
import frc.robot.utils.TurretAiming;

public class TurretShootAtHub extends Command {

    /** Amps above which we consider a ball to be passing through the feeder. */
    private static final double FEEDER_LOADED_CURRENT_AMPS = 5.0;

    /**
     * If no current spike is detected for this long, we consider all balls shot.
     * Resets every time a ball is detected.
     */
    private static final double NO_BALL_TIMEOUT_SECONDS = 2.5;

    /** Minimum run time before the finish condition is armed. */
    private static final double MIN_RUN_SECONDS = 0.5;

    private Translation2d ourHub = AimingConstants.Red_Hub;

    private final Timer runTimer      = new Timer();
    private final Timer noBallTimer   = new Timer();

    // Flipped to true when noBallTimer expires — checked by .until() in forAuto()
    private boolean done = false;

    public TurretShootAtHub() {
        addRequirements(Subsystems.turretAiming, Subsystems.turretFeeder, Subsystems.turretShooter);
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
        TurretState hubCalc = TurretAiming.calcState(AimingConstants.ShootingSamples, ourHub);

        Subsystems.turretAiming.setHoodTarget(hubCalc.hoodAngle());
        Subsystems.turretShooter.setSpeed(hubCalc.power());
        Subsystems.turretFeeder.popFuel(TurretFeederConstants.TARGET_SPEED);
        Subsystems.intake.setShootExtendAngle();

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
    public boolean isFinished() {
        return false; // never self-terminates — forAuto() uses .until(() -> done)
    }

    @Override
    public void end(boolean interrupted) {
        runTimer.stop();
        noBallTimer.stop();

        Subsystems.turretAiming.stop();
        Subsystems.turretShooter.stop();
        Subsystems.turretFeeder.stop();
        Subsystems.intake.setShootRetractAngle();
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