package org.firstinspires.ftc.teamcode.utils;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Shooter;

/**
 * Common Autonomous helpers used across multiple auton files.
 *
 * Minimal-change friendly: method names match your existing usage (BuildIntakeBurst, GoToPose, etc.).
 */
public class HelperUtils {

    private final ElapsedTime runtime;
    private final Telemetry telemetry;

    private final double intakeForwardSec;
    private final double intakeReverseSec;
    private long lastDsTelemMs = 0;
    private Limelight3A limelight;
    private int limelightSelectedPipeline = 0;

    public HelperUtils(ElapsedTime runtime,
                       Telemetry telemetry,
                       double intakeForwardSec,
                       double intakeReverseSec) {
        this.runtime = runtime;
        this.telemetry = telemetry;
        this.intakeForwardSec = intakeForwardSec;
        this.intakeReverseSec = intakeReverseSec;
    }

    // --- Limelight/vision tuning shared across helpers ---
    private double ll_kTurn = 0.015;     // rad/s per deg tx
    private double ll_txTolDeg = 1.0;    // deg
    private double ll_maxOmega = 2.0;    // rad/s
    private double ll_searchOmega = -1.2; // default (red)


    // ---- Cached last vision values (used by withPoseTelemetry) ----
    private boolean ll_lastValid = false;
    private double ll_lastTxDeg = Double.NaN;
    private double ll_lastOmegaAssist = 0.0;
    private long ll_lastVisionUpdateMs = 0;

    public void setLimelight(Limelight3A ll3A){
        this.limelight = ll3A;
    }

    public void setLimelightPipeline(int ll3APipeline){
        this.limelightSelectedPipeline = ll3APipeline;
    }

    public void setLimelightGains(double kTurn, double txTolDeg, double maxOmega) {
        this.ll_kTurn = kTurn;
        this.ll_txTolDeg = txTolDeg;
        this.ll_maxOmega = maxOmega;
    }

    //How precise the angle in degrees should be
    //“Start loose” (acquire fast): txTolDeg = 2.0
    //“Tighter” (more precise): txTolDeg = 1.0
    //“Very tight” (can oscillate): txTolDeg = 0.5
    public void setLimelightTxToleranceDeg(double txTolDeg) {
        this.ll_txTolDeg = txTolDeg;
    }


    public void setLimelightSearchOmega(double searchOmega) {
        this.ll_searchOmega = searchOmega;
    }

    public double getLimelightSearchOmega() {
        return ll_searchOmega;
    }



    // ==== Helpers =====================================

    public Action buildIntakeBurst(Intake intake, int numBalls) {
        Action[] steps = new Action[numBalls * 2];
        for (int i = 0; i < numBalls; i++) {
            steps[2 * i]     = intake.intakeForwardForTime(intakeReverseSec, -1.0);
            steps[2 * i + 1] = intake.intakeForwardForTime(intakeForwardSec,  1.0);
        }
        return new SequentialAction(steps);
    }

    // Generic helper: from any currentPose, build an Action that splines back to any other Pose.
    public Action GoToPose(Pose2d fromPose, MecanumDrive drive, Pose2d toPose) {
        if (toPose == null) {
            throw new IllegalStateException("shootingPose is not set before calling GoToPose()");
        }

        double travelTangent = Math.atan2(
                toPose.position.y - fromPose.position.y,
                toPose.position.x - fromPose.position.x
        );

        return drive.actionBuilder(fromPose)
                .splineToLinearHeading(toPose, travelTangent)
                .build();
    }

    public Action driveWithIntake(Action driveAction, Intake intake, double power) {
        return new Action() {
            private boolean started = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!started) {
                    intake.setPower(power);
                    started = true;
                }

                boolean driveStillRunning = driveAction.run(packet);

                if (!driveStillRunning) {
                    intake.setPower(0.0);
                }

                return driveStillRunning;
            }
        };
    }

    /**
     * Continuous Limelight tx correction blended into the follower via drive.setVisionOmega(...).
     *
     * Preconditions:
     *  - MecanumDrive has setVisionOmega(double) and setDrivePowers() blends it into angVel.
     */
    // Generic overload: caller supplies searchOmega sign/magnitude
    public Action driveWithLimelightTx(Action driveAction, MecanumDrive drive) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                Pose2d rrPose = drive.localizer.getPose();
                double robotYawDeg = Math.toDegrees(rrPose.heading.toDouble());
                limelight.updateRobotOrientation(robotYawDeg);

                LLResult result = limelight.getLatestResult();

                boolean valid = (result != null && result.isValid());
                double tx = valid ? result.getTx() : Double.NaN;

                double omegaAssist;
                if (!valid) {
                    omegaAssist = ll_searchOmega;
                } else if (Math.abs(tx) <= ll_txTolDeg) {
                    omegaAssist = 0.0;
                } else {
                    omegaAssist = -ll_kTurn * tx; // flip sign if diverges
                    if (omegaAssist >  ll_maxOmega) omegaAssist =  ll_maxOmega;
                    if (omegaAssist < -ll_maxOmega) omegaAssist = -ll_maxOmega;
                }

                // Cache + publish for telemetry (single source of truth)
                ll_lastOmegaAssist = omegaAssist;
                packet.put("LL_omegaAssist", omegaAssist);
                packet.put("LL_Valid", valid);
                packet.put("LL_tx_deg", tx);

                drive.setVisionOmega(omegaAssist);

                boolean stillRunning = driveAction.run(packet);

                if (!stillRunning) {
                    drive.setVisionOmega(0.0);
                    ll_lastOmegaAssist = 0.0;
                    packet.put("LL_omegaAssist", 0.0);
                }

                return stillRunning;
            }
        };
    }

    public Action alignToTagTx(MecanumDrive drive, double timeoutSec) {
        return new Action() {
            private boolean started = false;
            private long startMs;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!started) {
                    started = true;
                    startMs = System.currentTimeMillis();
                }

                // Use RR heading (deg) to update Limelight robot orientation
                Pose2d rrPose = drive.localizer.getPose();
                double yawDeg = Math.toDegrees(rrPose.heading.toDouble());
                limelight.updateRobotOrientation(yawDeg);

                LLResult result = limelight.getLatestResult();
                boolean valid = (result != null && result.isValid());
                double tx = valid ? result.getTx() : Double.NaN;

                double omegaAssist;
                if (!valid) {
                    omegaAssist = ll_searchOmega;          // search spin
                } else if (Math.abs(tx) <= ll_txTolDeg) {
                    omegaAssist = 0.0;                     // locked
                } else {
                    omegaAssist = -ll_kTurn * tx;          // P-correction
                    if (omegaAssist >  ll_maxOmega) omegaAssist =  ll_maxOmega;
                    if (omegaAssist < -ll_maxOmega) omegaAssist = -ll_maxOmega;
                }

                // Cache for withPoseTelemetry()
                ll_lastValid = valid;
                ll_lastTxDeg = tx;
                ll_lastOmegaAssist = omegaAssist;
                ll_lastVisionUpdateMs = System.currentTimeMillis();

                packet.put("LL_Valid", valid);
                packet.put("LL_tx_deg", tx);
                packet.put("LL_omegaAssist", omegaAssist);

                // Apply vision omega and command zero base motion.
                // Your setDrivePowers() will rotate at (0 + visionOmega).
                drive.setVisionOmega(omegaAssist);
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0));

                boolean locked = valid && (Math.abs(tx) <= ll_txTolDeg);
                boolean timedOut = (System.currentTimeMillis() - startMs) > (long) (timeoutSec * 1000.0);

                if (locked || timedOut) {
                    // Stop cleanly
                    drive.setVisionOmega(0.0);
                    drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0));
                    ll_lastOmegaAssist = 0.0;
                    return false; // done
                }

                return true; // keep aligning
            }
        };
    }


    public Action withPoseTelemetry(Action inner, MecanumDrive drive, Shooter shooter, boolean trapdoorOpen) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                shooter.update(trapdoorOpen);

                Pose2d pose = drive.localizer.getPose();
                double headingDeg = Math.toDegrees(pose.heading.toDouble());

                // ---- Minimal pose telemetry ----
                packet.put("X", pose.position.x);
                packet.put("Y", pose.position.y);
                packet.put("HeadingDeg", headingDeg);

                telemetry.addData("X", pose.position.x);
                telemetry.addData("Y", pose.position.y);
                telemetry.addData("Heading (deg)", headingDeg);

                // ---- Minimal Limelight telemetry ----
                if (limelight != null) {
                    telemetry.addData("LL Pipeline", ((limelightSelectedPipeline==0) ? "BLUE" : "RED"));
                    packet.put("LL_Pipeline_Selected", ((limelightSelectedPipeline==0) ? "BLUE" : "RED"));

                    // If vision wrapper hasn't updated recently, refresh valid/tx here (lightweight)
                    long now = System.currentTimeMillis();
                    if (now - ll_lastVisionUpdateMs > 80) { // ~12.5 Hz max refresh
                        limelight.updateRobotOrientation(headingDeg);
                        LLResult result = limelight.getLatestResult();
                        ll_lastValid = (result != null && result.isValid());
                        ll_lastTxDeg = ll_lastValid ? result.getTx() : Double.NaN;
                        ll_lastVisionUpdateMs = now;
                    }

                    telemetry.addData("LL Valid", ll_lastValid);
                    telemetry.addData("LL tx (deg)", ll_lastTxDeg);
                    telemetry.addData("LL omegaAssist", ll_lastOmegaAssist);

                    packet.put("LL_Valid", ll_lastValid);
                    packet.put("LL_tx_deg", ll_lastTxDeg);
                    packet.put("LL_omegaAssist", ll_lastOmegaAssist);
                }

                // ---- Throttle Driver Station updates (recommended for auton performance) ----
                long nowMs = System.currentTimeMillis();
                if (nowMs - lastDsTelemMs > 100) { // 10 Hz
                    telemetry.update();
                    lastDsTelemMs = nowMs;
                }

                return inner.run(packet);
            }
        };
    }
}
