package org.firstinspires.ftc.teamcode.utils;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.ShockwaveDecode_RRFieldCentric;
import org.firstinspires.ftc.teamcode.Shooter;

import java.util.List;

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


    // ---- Distance + aim control (auton) ----
    private double ll_kPDist = 0.02;          // power per cm (tune)
    private double ll_kPTurn = 0.015;         // rad/s per deg (you already have ll_kTurn; can reuse)
    private double ll_distDeadbandCm = 3.0;   // cm
    private double ll_txDeadbandDeg = 1.5;    // deg
    private double ll_maxFwdPower = 0.35;     // 0..1
    private double ll_maxTurnOmega = 1.2;     // rad/s (cap)
    private double ll_camForwardSign = 1.0;   // +1 or -1 depending on your “forward” convention

    // -------------------------------
    // Auto Align Controls (match TeleOp constants)
    // -------------------------------
    private double aa_CAMERA_FORWARD_SIGN = -1.0; // TeleOp: -1 if camera faces -X (flywheel-forward)
    private double aa_kP_DIST = 0.02;             // TeleOp kP_DIST
    private double aa_kP_TURN = 0.03;             // TeleOp kP_TURN (units: omega per deg)
    private double aa_MAX_FWD_POWER  = 0.60;      // TeleOp MAX_FWD_POWER
    private double aa_MAX_TURN_POWER = 0.45;      // TeleOp MAX_TURN_POWER
    private double aa_DIST_DEADBAND_CM = 1.5;     // TeleOp DIST_DEADBAND_CM
    private double aa_TX_DEADBAND_DEG  = 0.6;     // TeleOp TX_DEADBAND_DEG



    public void setLimelight(Limelight3A ll3A){
        this.limelight = ll3A;
    }

    public void setLimelightPipeline(int ll3APipeline){
            this.limelightSelectedPipeline = ll3APipeline;
            if (limelight != null) {
                limelight.pipelineSwitch(ll3APipeline);
            }
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

    public void setAutoAlignGains(double kPDist, double kPTurn,
                                  double distDeadbandCm, double txDeadbandDeg,
                                  double maxFwdPower, double maxTurnOmega,
                                  double camForwardSign) {
        this.ll_kPDist = kPDist;
        this.ll_kPTurn = kPTurn;
        this.ll_distDeadbandCm = distDeadbandCm;
        this.ll_txDeadbandDeg = txDeadbandDeg;
        this.ll_maxFwdPower = maxFwdPower;
        this.ll_maxTurnOmega = maxTurnOmega;
        this.ll_camForwardSign = camForwardSign;
    }

    public void setAutoAlignConstants(double cameraForwardSign,
                                      double kPDist, double kPTurn,
                                      double maxFwdPower, double maxTurnPower,
                                      double distDeadbandCm, double txDeadbandDeg) {
        aa_CAMERA_FORWARD_SIGN = cameraForwardSign;
        aa_kP_DIST = kPDist;
        aa_kP_TURN = kPTurn;
        aa_MAX_FWD_POWER = maxFwdPower;
        aa_MAX_TURN_POWER = maxTurnPower;
        aa_DIST_DEADBAND_CM = distDeadbandCm;
        aa_TX_DEADBAND_DEG = txDeadbandDeg;
    }





    // ==== Helpers =====================================

    public Action buildIntakeBurst(Intake intake, int numBalls) {
        Action[] steps = new Action[numBalls * 2];
        //Action[] steps = new Action[numBalls * 1];
        for (int i = 0; i < numBalls; i++) {
            steps[2 * i]     = intake.intakeForwardForTime(intakeReverseSec, -1.0);
            steps[2 * i + 1] = intake.intakeForwardForTime(intakeForwardSec,  1.0);
            //steps[i] = intake.intakeForwardForTime(intakeForwardSec,  1.0);
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


    public Action alignToTagTxAndDistanceCm(MecanumDrive drive,
                                            double targetDistanceCm,
                                            double timeoutSec) {
        return new Action() {
            private boolean started = false;
            private long startMs;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!started) {
                    started = true;
                    startMs = System.currentTimeMillis();
                }

                // Update Limelight yaw (deg)
                Pose2d rrPose = drive.localizer.getPose();
                double yawDeg = Math.toDegrees(rrPose.heading.toDouble());
                limelight.updateRobotOrientation(yawDeg);

                TagMeas tag = getBestTagMeasurement();

                double fwdCmd = 0.0;
                double omegaAssist = 0.0;

                if (tag.valid) {
                    // Distance: +err => too far
                    double distErr = tag.distanceCm - targetDistanceCm;

                    if (Math.abs(distErr) > aa_DIST_DEADBAND_CM) {
                        fwdCmd = clamp(
                                aa_CAMERA_FORWARD_SIGN * aa_kP_DIST * distErr,
                                -aa_MAX_FWD_POWER, aa_MAX_FWD_POWER
                        );
                    }

                    // Turn: tx>0 => tag right => turn CW (negative omega)
                    if (Math.abs(tag.txDeg) > aa_TX_DEADBAND_DEG) {
                        omegaAssist = clamp(
                                -aa_kP_TURN * tag.txDeg,
                                -aa_MAX_TURN_POWER, aa_MAX_TURN_POWER
                        );
                    } else {
                        omegaAssist = 0.0;
                    }
                } else {
                    // Search: rotate only
                    omegaAssist = ll_searchOmega;  // keep this small (0.2–0.5 recommended)
                    fwdCmd = 0.0;
                }

                // Cache for telemetry
                ll_lastValid = tag.valid;
                ll_lastTxDeg = tag.valid ? tag.txDeg : Double.NaN;
                ll_lastOmegaAssist = omegaAssist;
                ll_lastVisionUpdateMs = System.currentTimeMillis();

                packet.put("LL_Valid", tag.valid);
                packet.put("LL_tx_deg", tag.valid ? tag.txDeg : Double.NaN);
                packet.put("LL_dist_cm", tag.valid ? tag.distanceCm : Double.NaN);
                packet.put("LL_target_cm", targetDistanceCm);
                packet.put("LL_fwdCmd", fwdCmd);
                packet.put("LL_omegaAssist", omegaAssist);

                telemetry.addData("LL_Valid", tag.valid);
                telemetry.addData("LL_tx_deg", tag.valid ? tag.txDeg : Double.NaN);
                telemetry.addData("LL_dist_cm", tag.valid ? tag.distanceCm : Double.NaN);
                telemetry.addData("LL_target_cm", targetDistanceCm);
                telemetry.addData("LL_fwdCmd", fwdCmd);
                telemetry.addData("LL_omegaAssist", omegaAssist);

                // Apply using your blending model:
                drive.setVisionOmega(omegaAssist);
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(fwdCmd, 0.0), 0.0));

                boolean angleLocked = tag.valid && (Math.abs(tag.txDeg) <= aa_TX_DEADBAND_DEG);
                boolean distLocked  = tag.valid && (Math.abs(tag.distanceCm - targetDistanceCm) <= aa_DIST_DEADBAND_CM);
                boolean timedOut = (System.currentTimeMillis() - startMs) > (long)(timeoutSec * 1000.0);

                if ((angleLocked && distLocked) || timedOut) {
                    drive.setVisionOmega(0.0);
                    drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0));
                    ll_lastOmegaAssist = 0.0;
                    return false;
                }

                return true;
            }
        };
    }


    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private static class TagMeas {
        boolean valid;
        int id;
        double distanceCm; // lens -> tag
        double txDeg;      // horizontal offset
    }

    private TagMeas getBestTagMeasurement() {
        TagMeas out = new TagMeas();
        out.valid = false;

        if (limelight == null) return out;

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return out;

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) return out;

        // Choose the closest tag by 3D camera-space distance
        LLResultTypes.FiducialResult best = null;
        double bestDistM = Double.POSITIVE_INFINITY;

        for (LLResultTypes.FiducialResult f : fiducials) {
            Pose3D poseCam = f.getTargetPoseCameraSpace(); // target pose relative to CAMERA (lens origin)
            if (poseCam == null) continue;

            Position p = poseCam.getPosition();
            double xM = DistanceUnit.METER.fromUnit(p.unit, p.x);
            double yM = DistanceUnit.METER.fromUnit(p.unit, p.y);
            double zM = DistanceUnit.METER.fromUnit(p.unit, p.z);

            double distM = Math.sqrt(xM * xM + yM * yM + zM * zM);
            if (distM < bestDistM) {
                bestDistM = distM;
                best = f;
            }
        }

        if (best == null) return out;

        out.valid = true;
        out.id = best.getFiducialId();
        out.distanceCm = bestDistM * 100.0;
        out.txDeg = best.getTargetXDegrees();
        return out;
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
                packet.put("Run Time", runtime.toString());
                packet.put("Target RPM", shooter.getBangBangTargetVelocity());
                packet.put("Actual RPM", shooter.getActualRPM());
                packet.put("Trapdoor:", trapdoorOpen? "OPEN" : "CLOSED");

                telemetry.addData("X", pose.position.x);
                telemetry.addData("Y", pose.position.y);
                telemetry.addData("Heading (deg)", headingDeg);
                telemetry.addData("Run Time", runtime.toString());
                telemetry.addData("Target RPM", shooter.getBangBangTargetVelocity());
                telemetry.addData("Actual RPM", shooter.getActualRPM());
                telemetry.addData("Trapdoor:", trapdoorOpen? "OPEN" : "CLOSED");


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
