package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.util.Objects;

/**
 * Shockwave TeleOp using Pinpoint directly for field-centric heading (no Road Runner).
 * Replaces all Control Hub IMU usage with Pinpoint heading.
 */
@Disabled
@TeleOp(name = "ShockwaveDecode_PinpointDirect")
public class ShockwaveDecode_PinpointDirect_FieldCentric extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    // Drive Motors
    private DcMotor fl = null;
    private DcMotor fr = null;
    private DcMotor bl = null;
    private DcMotor br = null;

    // Mechanism Motors
    private DcMotor intake = null;
    private DcMotorEx shooterR = null;

    // Servo
    private Servo trapdoor = null;

    // Pinpoint
    private GoBildaPinpointDriver pinpoint;

    // --- State Variables for Toggles ---
    boolean flywheelOn = false;
    boolean a_button_previously_pressed = false;

    boolean trapdoorOpen = false;
    boolean y_button_previously_pressed = false;

    // --- CONTROL CONSTANTS ---
    static final double INTAKE_POWER = 1.0;

    // --- SHOOTER CONSTANTS (MEASURED / UPDATED) ---
    static final double ENCODER_COUNTS_PER_REV = 28.0;
    static final double MAX_TICKS_PER_SEC_MEASURED = 2400.0;
    static final double TARGET_FLYWHEEL_RPM = 2700.0;

    // PIDF for velocity control
    static final double shooterP = 5.0;
    static final double shooterI = 0.0;
    static final double shooterD = 0.0;
    static final double shooterF = 32767.0 / MAX_TICKS_PER_SEC_MEASURED;

    // Field-centric: driver heading offset (re-zero)
    private double headingOffset = 0.0;
    private boolean lastDpadUp = false;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        // --- HARDWARE MAPPING ---
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");

        intake = hardwareMap.get(DcMotor.class, "intake");
        shooterR = hardwareMap.get(DcMotorEx.class, "shooterR");

        trapdoor = hardwareMap.get(Servo.class, "trapdoor");
        trapdoor.setPosition(0.0); // start closed

        // --- MOTOR DIRECTION ---
        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);

        shooterR.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.FORWARD);

        // --- MOTOR BEHAVIOR ---
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // --- SHOOTER ENCODER + PIDF SETUP ---
        shooterR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterR.setVelocityPIDFCoefficients(shooterP, shooterI, shooterD, shooterF);

        // Drive without encoders
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // --- PINPOINT SETUP (mirrors your PinpointLocalizer) ---
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        final double inPerTick = MecanumDrive.PARAMS.inPerTick; // uses your tuned RR constant
        final double mmPerTick = inPerTick * 25.4;

        pinpoint.setEncoderResolution(1.0 / mmPerTick, DistanceUnit.MM);
        pinpoint.setOffsets(
                mmPerTick * PinpointLocalizer.PARAMS.parYTicks,
                mmPerTick * PinpointLocalizer.PARAMS.perpXTicks,
                DistanceUnit.MM
        );

        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );

        // Must be stationary
        pinpoint.resetPosAndIMU();

        telemetry.addData("Status", "Initialized. Ready to run!");
        telemetry.addData("Shooter CPR", ENCODER_COUNTS_PER_REV);
        telemetry.addData("Shooter max ticks/s", MAX_TICKS_PER_SEC_MEASURED);
        telemetry.addData("Shooter kF", "%.3f", shooterF);
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            // ----------------
            // PINPOINT UPDATE + HEADING
            // ----------------
            pinpoint.update();
            GoBildaPinpointDriver.DeviceStatus status = pinpoint.getDeviceStatus();

            // If Pinpoint isn't READY, stop driving (but allow mechanisms)
            boolean pinpointReady = Objects.requireNonNull(status) == GoBildaPinpointDriver.DeviceStatus.READY;

            double rawHeading = pinpointReady
                    ? pinpoint.getHeading(UnnormalizedAngleUnit.RADIANS)
                    : 0.0;

            // DPAD UP: re-zero field orientation
            boolean dpadUp = gamepad1.dpad_up;
            if (dpadUp && !lastDpadUp && pinpointReady) {
                headingOffset = rawHeading;
            }
            lastDpadUp = dpadUp;

            double fieldHeading = rawHeading - headingOffset;

            // ----------------
            // FIELD-CENTRIC MECANUM DRIVE (Gamepad 1 only)
            // ----------------
            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x; // right positive
            double turn = gamepad1.right_stick_x;

            // Deadzone
            if (Math.abs(forward) < 0.1) forward = 0;
            if (Math.abs(strafe) < 0.1) strafe = 0;
            if (Math.abs(turn) < 0.1) turn = 0;

            if (pinpointReady) {
                // Rotate joystick for field-centric control
                double rotatedX = strafe * Math.cos(-fieldHeading) - forward * Math.sin(-fieldHeading);
                double rotatedY = strafe * Math.sin(-fieldHeading) + forward * Math.cos(-fieldHeading);

                double denominator = Math.max(Math.abs(rotatedY) + Math.abs(rotatedX) + Math.abs(turn), 1.0);

                double flPower = (rotatedY + rotatedX + turn) / denominator;
                double blPower = (rotatedY - rotatedX + turn) / denominator;
                double frPower = (rotatedY - rotatedX - turn) / denominator;
                double brPower = (rotatedY + rotatedX - turn) / denominator;

                fl.setPower(flPower);
                bl.setPower(blPower);
                fr.setPower(frPower);
                br.setPower(brPower);
            } else {
                fl.setPower(0);
                fr.setPower(0);
                bl.setPower(0);
                br.setPower(0);
            }

            // ----------------
            // INTAKE CONTROL (Both Gamepads)
            // ----------------
            boolean intakeForwardTrigger = gamepad1.left_trigger > 0.1 || gamepad2.left_trigger > 0.1;
            boolean intakeReverseTrigger = gamepad1.right_trigger > 0.1 || gamepad2.right_trigger > 0.1;

            if (intakeReverseTrigger) {
                intake.setPower(-INTAKE_POWER);
            } else if (intakeForwardTrigger) {
                intake.setPower(INTAKE_POWER);
            } else {
                intake.setPower(0);
            }

            // ----------------
            // FLYWHEEL SHOOTER (Both Gamepads)
            // ----------------
            boolean aPressed = gamepad1.a || gamepad2.a;
            if (aPressed && !a_button_previously_pressed) {
                flywheelOn = !flywheelOn;
            }
            a_button_previously_pressed = aPressed;

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

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("--- Field Centric (Pinpoint Direct) ---", "");
            telemetry.addData("Pinpoint", "%s", status);
            telemetry.addData("Raw Heading (deg)", Math.toDegrees(rawHeading));
            telemetry.addData("Field Heading (deg)", Math.toDegrees(fieldHeading));
            telemetry.addData("X(in)", pinpoint.getPosX(DistanceUnit.INCH));
            telemetry.addData("Y(in)", pinpoint.getPosY(DistanceUnit.INCH));

            telemetry.addData("--- Shooter ---", "");
            telemetry.addData("Flywheel", flywheelOn ? "ON" : "OFF");
            telemetry.addData("Target RPM", TARGET_FLYWHEEL_RPM);
            telemetry.addData("Actual RPM", "%.1f", actualRPM);
            telemetry.addData("Actual ticks/s", "%.1f", actualTicksPerSec);
            telemetry.addData("Enc Pos", shooterR.getCurrentPosition());
            telemetry.addData("PIDF", "P=%.1f I=%.1f D=%.1f F=%.3f",
                    shooterP, shooterI, shooterD, shooterF);

            telemetry.update();
        }
    }
}
