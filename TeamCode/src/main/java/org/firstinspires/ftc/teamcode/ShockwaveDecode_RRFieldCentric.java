package org.firstinspires.ftc.teamcode;

//added as part of voltage fix
import com.qualcomm.robotcore.hardware.VoltageSensor;
import java.util.Iterator;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.List;

@TeleOp(name = "DECODE SEASON")
public class ShockwaveDecode_RRFieldCentric extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();

    // Mechanism Motors
    private DcMotor intake = null;
    private DcMotorEx shooterR = null;

    // Servo
    private Servo trapdoor = null;

    // --- State Variables for Toggles ---
    private boolean flywheelOn = false;
    private boolean aPrev = false;
    private boolean xPrev = false;

    // Y toggles drivetrain slow mode ONLY
    private boolean slowMode = false;
    private boolean yPressedLast = false;

    // Trapdoor toggle (moved off Y so Y can be slow-mode)
    private boolean trapdoorOpen = false;
    private boolean rbPrev = false;

    // --- CONTROL CONSTANTS ---
    private static final double INTAKE_POWER = 1.0;

    // --- TRAPDOOR POSITIONS (adjust to your robot) ---
    private static final double TRAPDOOR_CLOSED_POS = 1.0;
    private static final double TRAPDOOR_OPEN_POS   = 0.0;

    // --- DRIVETRAIN SLOW MODE MULTIPLIER ---
    private static final double DRIVE_SLOW_MULT = 0.50;

    // --- SHOOTER CONSTANTS ---
    private static final double ENCODER_COUNTS_PER_REV = 28.0;
    private static final double MAX_TICKS_PER_SEC_MEASURED = 2400.0;

    private enum FlywheelPreset { NEAR, FAR }

    private static final double RPM_NEAR = 2800;
    private static final double RPM_FAR  = 3200;

    private FlywheelPreset preset = FlywheelPreset.NEAR;
    // Default target RPM (A will set 2800, X will set 3500)
    private static double TARGET_FLYWHEEL_RPM = RPM_NEAR;

    // PIDF for velocity control
    private static final double shooterP = 8.0;
    private static final double shooterI = 0.0;
    private static final double shooterD = 0.0;
    private static final double shooterF = 13.84;//13.653 * 1.0431;
    // Controlling Feedforward based on the battery voltage, scale up when battery is low and scale down when its high
    private VoltageSensor batteryVoltageSensor;
    // Tune these once:
    private static final double V_BASE = 12.86;     // voltage you tuned F at (example)
    private static final double F_BASE = shooterF;   // was 13.653    // your tuned F at V_BASE (example)
    private static final double F_UPDATE_EPS = 0.05;

    private double lastFApplied = -1.0;



    // Field-centric: driver heading offset (re-zero)
    private double headingOffset = 0.0;
    private boolean lastDpadUp = false;

    // Optional: protect against sudden heading jumps
    private double lastRawHeading = 0.0;

    // -------------------------------
    // Limelight + Auto Align Controls
    // -------------------------------
    private Limelight3A limelight;

    // Pipeline selection during INIT
    private int selectedPipeline = 0;     // 0 or 1
    private boolean aPrevInit = false;    // edge detect for init toggle

    // +1 if camera faces +X (intake-forward). -1 if camera faces -X (flywheel-forward).
    private static final double CAMERA_FORWARD_SIGN = -1.0; // likely for your shooting setup


    private static double TARGET_DISTANCE_CM = 150.0; // set your desired shooting distance (lens->tag)
    // Hold LB to auto-drive to distance + auto-turn to center tag
    private static final double TARGET_DISTANCE_NEAR_CM = 150.0; // set your desired shooting distance (lens->tag)
    // Hold RB to auto-drive to distance + auto-turn to center tag
    private static final double TARGET_DISTANCE_FAR_CM = 308.0;

    // Simple P controllers (start here, then tune)
    private static final double kP_DIST = 0.02;   // power per cm error
    private static final double kP_TURN = 0.03;   // turn power per degree tx error

    private static final double MAX_FWD_POWER  = 0.60;
    private static final double MAX_TURN_POWER = 0.45;

    private static final double DIST_DEADBAND_CM = 1.5;
    private static final double TX_DEADBAND_DEG  = 0.6;

    private double getPresetRpm() {
        return (preset == FlywheelPreset.NEAR) ? RPM_NEAR : RPM_FAR;
    }

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        // Road Runner drive (assumes your MecanumDrive uses PinpointLocalizer already)
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        // --- HARDWARE MAPPING (mechanisms only) ---
        intake = hardwareMap.get(DcMotor.class, "intake");
        shooterR = hardwareMap.get(DcMotorEx.class, "shooterR");
        trapdoor = hardwareMap.get(Servo.class, "trapdoor");

        Iterator<VoltageSensor> it = hardwareMap.voltageSensor.iterator();
        if (it.hasNext()) {
            batteryVoltageSensor = it.next();
        }


        // Start closed
        trapdoorOpen = false;
        trapdoor.setPosition(TRAPDOOR_CLOSED_POS);

        shooterR.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.FORWARD);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // --- SHOOTER SETUP ---
        shooterR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients pidf = new PIDFCoefficients(shooterP, shooterI, shooterD, shooterF, MotorControlAlgorithm.PIDF);
        shooterR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // --- LIMELIGHT SETUP (INIT selection) ---
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);

        // Default pipeline
        selectedPipeline = 0;
        limelight.pipelineSwitch(selectedPipeline);

        // Start Limelight during INIT so you can see tags / distance before Play
        limelight.start();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Shooter kF", "%.3f", shooterF);
        telemetry.update();

        // ------------------------
        // INIT LOOP: choose pipeline 0 or 1 before pressing Play
        // ------------------------
        while (!isStarted() && !isStopRequested()) {
            boolean aNowInit = gamepad1.a;
            if (aNowInit && !aPrevInit) {
                selectedPipeline = (selectedPipeline == 0) ? 1 : 0;
                limelight.pipelineSwitch(selectedPipeline);
            }
            aPrevInit = aNowInit;

            TagMeas tag = getBestTagMeasurement();

            telemetry.addLine("INIT: Press A to toggle Limelight pipeline (0:BLUE / 1:RED)");
            telemetry.addData("Selected Pipeline", ((selectedPipeline == 0)? "BLUE" : "RED"));
            telemetry.addData("LB AutoAlign", "Hold during TeleOp");
            telemetry.addData("Target Dist (cm)", "%.1f", TARGET_DISTANCE_CM);

            if (tag.valid) {
                telemetry.addData("Tag ID", tag.id);
                telemetry.addData("Tag dist (cm)", "%.1f", tag.distanceCm);
                telemetry.addData("Tag tx (deg)", "%.2f", tag.txDeg);
            } else {
                telemetry.addLine("No valid AprilTag (or Full 3D disabled / no target).");
            }

            telemetry.addData("Note", "Press PLAY to start");
            telemetry.update();
            idle();
        }

        waitForStart();
        runtime.reset();

        // Initialize heading offset to current heading at start
        drive.updatePoseEstimate();
        lastRawHeading = drive.localizer.getPose().heading.toDouble();
        headingOffset = lastRawHeading;

        while (opModeIsActive()) {
            // Update RR pose/localizer (PinpointLocalizer.update() runs inside)
            drive.updatePoseEstimate();

            // Heading from Pinpoint via RR localizer
            double rawHeading = drive.localizer.getPose().heading.toDouble();

            // Optional: continuity protection if heading ever jumps hard
            double dH = rawHeading - lastRawHeading;
            if (Math.abs(dH) > Math.toRadians(90)) {
                headingOffset += dH;
            }
            lastRawHeading = rawHeading;

            // Dpad up: re-zero "field forward" to wherever robot is currently facing
            boolean dpadUp = gamepad1.dpad_up;
            if (dpadUp && !lastDpadUp) {
                headingOffset = rawHeading;
            }
            lastDpadUp = dpadUp;

            double fieldHeading = rawHeading - headingOffset;

            // ----------------
            // Y TOGGLE: SLOW MODE (DRIVETRAIN ONLY)
            // ----------------
            boolean yNow = gamepad1.y;
            if (yNow && !yPressedLast) {
                slowMode = !slowMode;
            }
            yPressedLast = yNow;

            double driveMult = slowMode ? DRIVE_SLOW_MULT : 1.0;

            // ----------------
            // MANUAL FIELD-CENTRIC DRIVE INPUTS
            // ----------------
            double forward = -gamepad1.left_stick_y;     // + forward
            double strafe  =  gamepad1.left_stick_x;     // + right (driver)
            double turn    = -gamepad1.right_stick_x;    // stick right -> turn right (clockwise)

            // Deadzones (apply before scaling)
            if (Math.abs(forward) < 0.1) forward = 0;
            if (Math.abs(strafe)  < 0.1) strafe  = 0;
            if (Math.abs(turn)    < 0.1) turn    = 0;

            // Apply slow mode to drivetrain ONLY
            forward *= driveMult;
            strafe  *= driveMult;
            turn    *= driveMult;

            // Convert driver strafe (right+) into RR +Y LEFT
            double fieldX = forward;      // +X forward
            double fieldYLeft = -strafe;  // +Y left (stick right => negative)

            // Rotate field -> robot by -fieldHeading
            double cos = Math.cos(-fieldHeading);
            double sin = Math.sin(-fieldHeading);

            double robotX = fieldX * cos - fieldYLeft * sin;      // forward
            double robotYLeft = fieldX * sin + fieldYLeft * cos;  // left

            // ----------------
            // AUTO ALIGN (hold LB): NEAR distance + aiming
            // AUTO ALIGN (hold RB): FAR distance + aiming
            // ----------------
            boolean autoAlignLB = gamepad1.left_bumper || gamepad2.left_bumper; // hold to engage
            if(autoAlignLB){
                TARGET_DISTANCE_CM = TARGET_DISTANCE_NEAR_CM;
            }
            boolean autoAlignRB = gamepad1.right_bumper || gamepad2.right_bumper;
            if(autoAlignRB){
                TARGET_DISTANCE_CM = TARGET_DISTANCE_FAR_CM;
            }

            TagMeas tag = getBestTagMeasurement();

            if ((autoAlignLB || autoAlignRB) && tag.valid) {
                // Distance controller: positive error means too far -> drive forward (-X flywheel facing)
                double distErr = tag.distanceCm - TARGET_DISTANCE_CM;  // positive => too far

                double fwdCmd = 0.0;
                if (Math.abs(distErr) > DIST_DEADBAND_CM) {
                    fwdCmd  = clamp(CAMERA_FORWARD_SIGN * kP_DIST * distErr,
                            -MAX_FWD_POWER, MAX_FWD_POWER);
                }

                // Turn controller: tx>0 means tag to the right -> turn clockwise.
                // Your convention: clockwise = negative omega, so use negative sign.
                double turnCmd = 0.0;
                if (Math.abs(tag.txDeg) > TX_DEADBAND_DEG) {
                    turnCmd = clamp(-kP_TURN * tag.txDeg, -MAX_TURN_POWER, MAX_TURN_POWER);
                }

                // Optional: allow driver to strafe while auto-align runs.
                // This strafe is ROBOT-CENTRIC (not field-centric) for predictability while aiming.
                double strafeDriver = gamepad1.left_stick_x;
                if (Math.abs(strafeDriver) < 0.1) strafeDriver = 0;

                // Map stick right+ to RR +Y left (so negate)
                double strafeCmdYLeft = (-strafeDriver) * driveMult;

                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(fwdCmd, strafeCmdYLeft), turnCmd));

                telemetry.addLine("AUTO ALIGN (Hold LB)");
                telemetry.addData("Tag ID", tag.id);
                telemetry.addData("Tag dist (cm)", "%.1f", tag.distanceCm);
                telemetry.addData("Tag dist (inch)", "%.1f", tag.distanceCm/2.54);
                telemetry.addData("Tag tx (deg)", "%.2f", tag.txDeg);
                telemetry.addData("distErr (cm)", "%.1f", (TARGET_DISTANCE_CM - tag.distanceCm));
                //telemetry.addData("cmd fwd/turn", "%.2f / %.2f", fwdCmd, turnCmd);
                //telemetry.addData("fwdCmd", "%.2f", fwdCmd);

            } else {
                // Manual field-centric drive
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(robotX, robotYLeft), turn));
            }

            // ----------------
            // INTAKE (either gamepad)
            // ----------------
            boolean intakeForward = (gamepad1.left_trigger > 0.1) || (gamepad2.left_trigger > 0.1);
            boolean intakeReverse = (gamepad1.right_trigger > 0.1) || (gamepad2.right_trigger > 0.1);

            if (intakeReverse) {
                intake.setPower(-INTAKE_POWER);
            } else if (intakeForward) {
                intake.setPower(INTAKE_POWER);
            } else {
                intake.setPower(0);
            }

            // ----------------
            // TRAPDOOR TOGGLE (RIGHT BUMPER)
            // ----------------
//            boolean rbNow = gamepad1.right_bumper || gamepad2.right_bumper;
//            if (rbNow && !rbPrev) {
//                trapdoorOpen = !trapdoorOpen;
//                trapdoor.setPosition(trapdoorOpen ? TRAPDOOR_OPEN_POS : TRAPDOOR_CLOSED_POS);
//            }
//            rbPrev = rbNow;

            boolean dplNow = gamepad1.dpad_left || gamepad2.dpad_left;
            if (dplNow) {
                trapdoorOpen = true;
                trapdoor.setPosition(TRAPDOOR_OPEN_POS);
            }
            boolean dprNow = gamepad1.dpad_right || gamepad2.dpad_right;
            if(dprNow) {
                trapdoorOpen = false;
                trapdoor.setPosition(TRAPDOOR_CLOSED_POS);
            }

            // ----------------
            // FLYWHEEL PRESETS + TOGGLE (A/X)
            // ----------------
            boolean aNow = gamepad1.a || gamepad2.a;
            boolean xNow = gamepad1.x || gamepad2.x;

            boolean aEdge = aNow && !aPrev;
            boolean xEdge = xNow && !xPrev;

//            if (xEdge) {
//                if (flywheelOn && TARGET_FLYWHEEL_RPM == 3500) {
//                    flywheelOn = false;
//                } else {
//                    TARGET_FLYWHEEL_RPM = 3500;
//                    flywheelOn = true;
//                }
//            } else if (aEdge) {
//                if (flywheelOn && TARGET_FLYWHEEL_RPM >= 2800) {
//                    flywheelOn = false;
//                } else {
//                    TARGET_FLYWHEEL_RPM = 2950;
//                    flywheelOn = true;
//                }
//            }

            if (xEdge) {
                if (flywheelOn && preset == FlywheelPreset.FAR) {
                    flywheelOn = false;
                } else {
                    preset = FlywheelPreset.FAR;
                    flywheelOn = true;
                }
            } else if (aEdge) {
                if (flywheelOn && preset == FlywheelPreset.NEAR) {
                    flywheelOn = false;
                } else {
                    preset = FlywheelPreset.NEAR;
                    flywheelOn = true;
                }
            }

            aPrev = aNow;
            xPrev = xNow;

//            if (flywheelOn) {
//                double targetTicksPerSec = (TARGET_FLYWHEEL_RPM / 60.0) * ENCODER_COUNTS_PER_REV;
//                shooterR.setVelocity(targetTicksPerSec);
//            } else {
//                shooterR.setVelocity(0);
//            }

            if (flywheelOn) {
                applyShooterPIDFWithVoltageComp(shooterR);
                TARGET_FLYWHEEL_RPM = getPresetRpm();
                double targetTicksPerSec = (TARGET_FLYWHEEL_RPM / 60.0) * ENCODER_COUNTS_PER_REV;
                shooterR.setVelocity(targetTicksPerSec);
            } else {
                shooterR.setVelocity(0);
            }

            // ----------------
            // TELEMETRY (existing + a bit more)
            // ----------------
            double actualTicksPerSec = shooterR.getVelocity();
            double actualRPM = (actualTicksPerSec / ENCODER_COUNTS_PER_REV) * 60.0;

            telemetry.addData("Trapdoor", trapdoorOpen ? "OPEN" : "CLOSED");
            telemetry.addData("Alliance Selected:", ((selectedPipeline == 0)? "BLUE" : "RED"));
            telemetry.addData("Target RPM", TARGET_FLYWHEEL_RPM);
            telemetry.addData("Actual RPM", "%.1f", actualRPM);
            telemetry.addData("Slow Mode (Y)", slowMode ? "ON (50%)" : "OFF (100%)");
            telemetry.addData("Run Time", runtime.toString());
            //telemetry.addData("Limelight Pipeline", selectedPipeline);
            //telemetry.addData("Heading raw (deg)", Math.toDegrees(rawHeading));
            //telemetry.addData("Heading field (deg)", Math.toDegrees(fieldHeading));
            //telemetry.addData("Stick f/s/t", "%.2f %.2f %.2f",forward, strafe, -gamepad1.right_stick_x);

            telemetry.addData("Flywheel", flywheelOn ? "ON" : "OFF");


            //PIDFCoefficients read = shooterR.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
            //telemetry.addData("PIDF readback", "P=%.2f I=%.2f D=%.2f F=%.2f", read.p, read.i, read.d, read.f);

            //double targetTicksPerSec = (TARGET_FLYWHEEL_RPM / 60.0) * ENCODER_COUNTS_PER_REV;
            //telemetry.addData("Target ticks/s", "%.1f", targetTicksPerSec);
            //telemetry.addData("Actual ticks/s", "%.1f", shooterR.getVelocity());
            //telemetry.addData("Error ticks/s", "%.1f", targetTicksPerSec - shooterR.getVelocity());

            // Helpful: show tag even when not auto-aligning
            if (tag.valid) {
                telemetry.addData("Tag ID", tag.id);
                telemetry.addData("Tag dist (cm)", "%.1f", tag.distanceCm);
                telemetry.addData("Tag dist (inch)", "%.1f", tag.distanceCm/2.54);
                telemetry.addData("Tag tx (deg)", "%.2f", tag.txDeg);
            } else {
                telemetry.addLine("Tag: none");
            }

            telemetry.update();
        }
    }

    private void applyShooterPIDFWithVoltageComp(DcMotorEx shooter) {
        if (batteryVoltageSensor == null) return;

        double vNow = batteryVoltageSensor.getVoltage();
        if (vNow < 1.0) vNow = V_BASE; // safety fallback

        // Scale feedforward as voltage drops: F increases
        double fScaled = F_BASE * (V_BASE / vNow);

        PIDFCoefficients pidf = new PIDFCoefficients(shooterP, shooterI, shooterD, fScaled, MotorControlAlgorithm.PIDF);

        // Avoid spamming the hub every loop
        if (lastFApplied < 0 || Math.abs(fScaled - lastFApplied) > F_UPDATE_EPS) {
            //shooter.setVelocityPIDFCoefficients(shooterP, shooterI, shooterD, fScaled);
            shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
            lastFApplied = fScaled;
        }
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
}


/*package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@TeleOp(name = "DECODE SEASON")
public class ShockwaveDecode_RRFieldCentric extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();

    // Mechanism Motors
    private DcMotor intake = null;
    private DcMotorEx shooterR = null;

    // Servo
    private Servo trapdoor = null;

    // --- State Variables for Toggles ---
    private boolean flywheelOn = false;
    private boolean aPrev = false;
    private boolean xPrev = false;

    // Y toggles drivetrain slow mode ONLY
    private boolean slowMode = false;
    private boolean yPressedLast = false;

    // Trapdoor toggle (moved off Y so Y can be slow-mode)
    private boolean trapdoorOpen = false;
    private boolean rbPrev = false;

    // --- CONTROL CONSTANTS ---
    private static final double INTAKE_POWER = 1.0;

    // --- TRAPDOOR POSITIONS (adjust to your robot) ---
    private static final double TRAPDOOR_CLOSED_POS = 1.0;
    private static final double TRAPDOOR_OPEN_POS   = 0.0;

    // --- DRIVETRAIN SLOW MODE MULTIPLIER ---
    private static final double DRIVE_SLOW_MULT = 0.30;

    // --- SHOOTER CONSTANTS ---
    private static final double ENCODER_COUNTS_PER_REV = 28.0;
    private static final double MAX_TICKS_PER_SEC_MEASURED = 2400.0;

    // Default target RPM (A will set 2800, X will set 3500)
    private static double TARGET_FLYWHEEL_RPM = 2800;

    // PIDF for velocity control
    private static final double shooterP = 8.0;
    private static final double shooterI = 0.0;
    private static final double shooterD = 0.0;
    private static final double shooterF = 13.653; // (32767.0 / MAX_TICKS_PER_SEC_MEASURED) * 1.064 = 14.53 //(32767.0 / MAX_TICKS_PER_SEC_MEASURED) * 1.106 = 15.10 // 13.653

    // Field-centric: driver heading offset (re-zero)
    private double headingOffset = 0.0;
    private boolean lastDpadUp = false;

    // Optional: protect against sudden heading jumps
    private double lastRawHeading = 0.0;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        // Road Runner drive (assumes your MecanumDrive uses PinpointLocalizer already)
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        // --- HARDWARE MAPPING (mechanisms only) ---
        intake = hardwareMap.get(DcMotor.class, "intake");
        shooterR = hardwareMap.get(DcMotorEx.class, "shooterR");
        trapdoor = hardwareMap.get(Servo.class, "trapdoor");

        // Start closed
        trapdoorOpen = false;
        trapdoor.setPosition(TRAPDOOR_CLOSED_POS);

        shooterR.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.FORWARD);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // --- SHOOTER SETUP ---
        shooterR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //shooterR.setVelocityPIDFCoefficients(shooterP, shooterI, shooterD, shooterF);
        // Use explicit PIDF set for RUN_USING_ENCODER
        PIDFCoefficients pidf = new PIDFCoefficients(shooterP, shooterI, shooterD, shooterF, MotorControlAlgorithm.PIDF);
        shooterR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Shooter kF", "%.3f", shooterF);
        telemetry.update();

        waitForStart();
        runtime.reset();

        // Initialize heading offset to current heading at start
        drive.updatePoseEstimate();
        lastRawHeading = drive.localizer.getPose().heading.toDouble();
        headingOffset = lastRawHeading;

        while (opModeIsActive()) {
            // Update RR pose/localizer (PinpointLocalizer.update() runs inside)
            drive.updatePoseEstimate();

            // Heading from Pinpoint via RR localizer
            double rawHeading = drive.localizer.getPose().heading.toDouble();

            // Optional: continuity protection if heading ever jumps hard
            double dH = rawHeading - lastRawHeading;
            if (Math.abs(dH) > Math.toRadians(90)) {
                headingOffset += dH;
            }
            lastRawHeading = rawHeading;

            // Dpad up: re-zero "field forward" to wherever robot is currently facing
            boolean dpadUp = gamepad1.dpad_up;
            if (dpadUp && !lastDpadUp) {
                headingOffset = rawHeading;
            }
            lastDpadUp = dpadUp;

            double fieldHeading = rawHeading - headingOffset;

            // ----------------
            // Y TOGGLE: SLOW MODE (DRIVETRAIN ONLY)
            // ----------------
            boolean yNow = gamepad1.y;
            if (yNow && !yPressedLast) {
                slowMode = !slowMode;
            }
            yPressedLast = yNow;

            double driveMult = slowMode ? DRIVE_SLOW_MULT : 1.0;

            // ----------------
            // FIELD-CENTRIC DRIVE (RR convention: +X forward, +Y LEFT)
            // ----------------
            double forward = -gamepad1.left_stick_y;     // + forward
            double strafe  =  gamepad1.left_stick_x;     // + right (driver)
            double turn    = -gamepad1.right_stick_x;    // stick right -> turn right (clockwise)

            // Deadzones (apply before scaling)
            if (Math.abs(forward) < 0.1) forward = 0;
            if (Math.abs(strafe)  < 0.1) strafe  = 0;
            if (Math.abs(turn)    < 0.1) turn    = 0;

            // Apply slow mode to drivetrain ONLY
            forward *= driveMult;
            strafe  *= driveMult;
            turn    *= driveMult;

            // Convert driver strafe (right+) into RR +Y LEFT
            double fieldX = forward;      // +X forward
            double fieldYLeft = -strafe;  // +Y left (stick right => negative)

            // Rotate field -> robot by -fieldHeading
            double cos = Math.cos(-fieldHeading);
            double sin = Math.sin(-fieldHeading);

            double robotX = fieldX * cos - fieldYLeft * sin;      // forward
            double robotYLeft = fieldX * sin + fieldYLeft * cos;  // left

            // Pass robotYLeft directly into RR vector (do NOT flip to "right")
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(robotX, robotYLeft), turn));

            // ----------------
            // INTAKE (either gamepad)
            // ----------------
            boolean intakeForward = (gamepad1.left_trigger > 0.1) || (gamepad2.left_trigger > 0.1);
            boolean intakeReverse = (gamepad1.right_trigger > 0.1) || (gamepad2.right_trigger > 0.1);

            if (intakeReverse) {
                intake.setPower(-INTAKE_POWER);
            } else if (intakeForward) {
                intake.setPower(INTAKE_POWER);
            } else {
                intake.setPower(0);
            }

            // ----------------
            // TRAPDOOR TOGGLE (RIGHT BUMPER)
            // ----------------
            boolean rbNow = gamepad1.right_bumper || gamepad2.right_bumper;
            if (rbNow && !rbPrev) {
                trapdoorOpen = !trapdoorOpen;
                trapdoor.setPosition(trapdoorOpen ? TRAPDOOR_OPEN_POS : TRAPDOOR_CLOSED_POS);
            }
            rbPrev = rbNow;

            // ----------------
            // FLYWHEEL PRESETS + TOGGLE (A/X)
            //  - A: 2800 RPM preset with toggle behavior
            //  - X: 3500 RPM preset with toggle behavior
            // Rule:
            //   If flywheel is ON and same button pressed is pressed again -> toggle flywheel OFF
            //   Else -> set preset and turn it ON
            //   If A is pressed and then X is pressed, the rpm jumps from 2800  to 3500 and vice versa
            // ----------------
            boolean aNow = gamepad1.a || gamepad2.a;
            boolean xNow = gamepad1.x || gamepad2.x;

            boolean aEdge = aNow && !aPrev;
            boolean xEdge = xNow && !xPrev;

            if (xEdge) {
                if (flywheelOn && TARGET_FLYWHEEL_RPM == 3500) {
                    flywheelOn = false;              // X pressed again at 3500 -> OFF
                } else {
                    TARGET_FLYWHEEL_RPM = 3500;      // otherwise -> set 3500 and ON
                    flywheelOn = true;
                }
            } else if (aEdge) {
                if (flywheelOn && TARGET_FLYWHEEL_RPM == 2800) {
                    flywheelOn = false;              // A pressed again at 2800 -> OFF
                } else {
                    TARGET_FLYWHEEL_RPM = 2800;      // otherwise -> set 2800 and ON
                    flywheelOn = true;
                }
            }

            aPrev = aNow;
            xPrev = xNow;


            if (flywheelOn) {
                double targetTicksPerSec = (TARGET_FLYWHEEL_RPM / 60.0) * ENCODER_COUNTS_PER_REV;
                shooterR.setVelocity(targetTicksPerSec);
            } else {
                shooterR.setVelocity(0);
            }

            // ----------------
            // TELEMETRY
            // ----------------
            double actualTicksPerSec = shooterR.getVelocity();
            double actualRPM = (actualTicksPerSec / ENCODER_COUNTS_PER_REV) * 60.0;

            telemetry.addData("Run Time", runtime.toString());
            telemetry.addData("Slow Mode (Y)", slowMode ? "ON (30%)" : "OFF (100%)");
            telemetry.addData("Heading raw (deg)", Math.toDegrees(rawHeading));
            telemetry.addData("Heading field (deg)", Math.toDegrees(fieldHeading));
            telemetry.addData("Stick f/s/t", "%.2f %.2f %.2f",
                    forward, strafe, -gamepad1.right_stick_x);
            telemetry.addData("Flywheel", flywheelOn ? "ON" : "OFF");
            telemetry.addData("Target RPM", TARGET_FLYWHEEL_RPM);
            telemetry.addData("Actual RPM", "%.1f", actualRPM);
            telemetry.addData("Trapdoor", trapdoorOpen ? "OPEN" : "CLOSED");

            PIDFCoefficients read = shooterR.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
            telemetry.addData("PIDF readback", "P=%.2f I=%.2f D=%.2f F=%.2f", read.p, read.i, read.d, read.f);
            telemetry.addData("Shooter mode", shooterR.getMode());

            double targetTicksPerSec = (TARGET_FLYWHEEL_RPM / 60.0) * ENCODER_COUNTS_PER_REV;
            telemetry.addData("Target ticks/s", "%.1f", targetTicksPerSec);
            telemetry.addData("Actual ticks/s", "%.1f", shooterR.getVelocity());
            telemetry.addData("Error ticks/s", "%.1f", targetTicksPerSec - shooterR.getVelocity());


            telemetry.update();
        }
    }
}
*/