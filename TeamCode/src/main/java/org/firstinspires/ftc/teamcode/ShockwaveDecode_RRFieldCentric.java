package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    // Y toggles drivetrain slow mode ONLY
    private boolean slowMode = false;
    private boolean yPressedLast = false;

    // Trapdoor toggle (moved off Y so Y can be slow-mode)
    private boolean trapdoorOpen = false;
    private boolean rbPrev = false;

    // --- CONTROL CONSTANTS ---
    private static final double INTAKE_POWER = 1.0;

    // --- TRAPDOOR POSITIONS (adjust to your robot) ---
    private static final double TRAPDOOR_CLOSED_POS = 0.0;
    private static final double TRAPDOOR_OPEN_POS   = 1.0;

    // --- DRIVETRAIN SLOW MODE MULTIPLIER ---
    private static final double DRIVE_SLOW_MULT = 0.30;

    // --- SHOOTER CONSTANTS ---
    private static final double ENCODER_COUNTS_PER_REV = 28.0;
    private static final double MAX_TICKS_PER_SEC_MEASURED = 2400.0;
    private static final double TARGET_FLYWHEEL_RPM = 2800.0;

    // PIDF for velocity control
    private static final double shooterP = 7.0;
    private static final double shooterI = 3.0;
    private static final double shooterD = 1.5;
    private static final double shooterF = 32767.0 / MAX_TICKS_PER_SEC_MEASURED;

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
        shooterR.setVelocityPIDFCoefficients(shooterP, shooterI, shooterD, shooterF);

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
            // FLYWHEEL TOGGLE (A) + Velocity control
            // ----------------
            boolean aPressed = gamepad1.a || gamepad2.a;
            if (aPressed && !aPrev) {
                flywheelOn = !flywheelOn;
            }
            aPrev = aPressed;

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
            telemetry.update();
        }
    }
}
