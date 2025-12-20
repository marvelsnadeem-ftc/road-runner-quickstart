package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

@Disabled
@TeleOp
public class ShockwaveDecode extends LinearOpMode {

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

    // IMU
    private IMU imu;

    // --- State Variables for Toggles ---
    boolean flywheelOn = false;
    boolean a_button_previously_pressed = false;

    boolean trapdoorOpen = false;
    boolean y_button_previously_pressed = false;

    // -------------------------------------------------------------------------
    // FLYWHEEL / ENCODER FACTS (MEASURED)
    // -------------------------------------------------------------------------
    // Flywheel wheel: goBILDA Hogback traction wheel, 96mm diameter (direct mounted).
    //
    // SWYFT Spike motor encoder spec says 28 PPR, but rather than assuming 28*4, we MEASURED:
    // - Rotate wheel exactly 10 revolutions by hand
    // - Telemetry showed 280 counts
    // => Encoder counts per revolution (CPR) = 280 / 10 = 28 counts/rev (for this setup)
    //
    // Velocity in FTC SDK is ticks/second.
    // RPM <-> ticks/s:
    //   ticksPerSec = (RPM / 60) * CPR
    //   RPM = (ticksPerSec / CPR) * 60
    //
    // We also MEASURED max velocity with full power:
    //   maxTicksPerSec_measured = 2400 ticks/sec
    // Feedforward:
    //   kF = 32767 / maxTicksPerSec_measured = 32767 / 2400 = 13.653
    // -------------------------------------------------------------------------

    // --- CONTROL CONSTANTS ---
    static final double INTAKE_POWER = 1.0;

    // --- SHOOTER CONSTANTS (MEASURED / UPDATED) ---
    static final double ENCODER_COUNTS_PER_REV = 28.0;   // measured CPR
    static final double MAX_TICKS_PER_SEC_MEASURED = 2400.0; // measured max getVelocity()

    // Choose your desired flywheel speed in RPM (tune this for scoring consistency)
    static final double TARGET_FLYWHEEL_RPM = 2700.0;

    // PIDF for velocity control
    // Start with I=0 for flywheels. Tune P upward as needed; add D only if oscillation occurs.
    static final double shooterP = 5.0;     // starting point; tune
    static final double shooterI = 0.0;
    static final double shooterD = 0.0;
    static final double shooterF = 32767.0 / MAX_TICKS_PER_SEC_MEASURED; // 13.653

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

        // IMU orientation: Logo UP, USB BACK
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                )
        );
        imu.initialize(parameters);
        imu.resetYaw();

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

        // PIDF for velocity control (ticks/sec)
        shooterR.setVelocityPIDFCoefficients(shooterP, shooterI, shooterD, shooterF);

        // Drive without encoders (field-centric uses IMU)
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Initialized. Ready to run!");
        telemetry.addData("Shooter CPR", ENCODER_COUNTS_PER_REV);
        telemetry.addData("Shooter max ticks/s", MAX_TICKS_PER_SEC_MEASURED);
        telemetry.addData("Shooter kF", "%.3f", shooterF);
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            //----------------//
            // FIELD-CENTRIC MECANUM DRIVE (Gamepad 1 only)
            //----------------//
            double y = -gamepad1.left_stick_y;   // Forward/Backward
            double x = gamepad1.left_stick_x;    // Strafe
            double rx = gamepad1.right_stick_x;  // Rotation

            // Deadzone
            if (Math.abs(y) < 0.1) y = 0;
            if (Math.abs(x) < 0.1) x = 0;
            if (Math.abs(rx) < 0.1) rx = 0;

            // Get current heading
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            double botHeading = orientation.getYaw(AngleUnit.RADIANS);

            // Rotate joystick for field-centric control
            double rotatedX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotatedY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            double denominator = Math.max(Math.abs(rotatedY) + Math.abs(rotatedX) + Math.abs(rx), 1);
            double flPower = (rotatedY + rotatedX + rx) / denominator;
            double blPower = (rotatedY - rotatedX + rx) / denominator;
            double frPower = (rotatedY - rotatedX - rx) / denominator;
            double brPower = (rotatedY + rotatedX - rx) / denominator;

            fl.setPower(flPower);
            bl.setPower(blPower);
            fr.setPower(frPower);
            br.setPower(brPower);

            // Press DPAD UP to reset field orientation
            if (gamepad1.dpad_up) {
                imu.resetYaw();
            }

            //----------------//
            // INTAKE CONTROL (Both Gamepads)
            //----------------//
            boolean intakeForwardTrigger = gamepad1.left_trigger > 0.1 || gamepad2.left_trigger > 0.1;
            boolean intakeReverseTrigger = gamepad1.right_trigger > 0.1 || gamepad2.right_trigger > 0.1;

            if (intakeReverseTrigger) {
                intake.setPower(-INTAKE_POWER);
            } else if (intakeForwardTrigger) {
                intake.setPower(INTAKE_POWER);
            } else {
                intake.setPower(0);
            }

            //----------------//
            // FLYWHEEL SHOOTER (Both Gamepads) - Velocity control in ticks/sec
            //----------------//
            boolean aPressed = gamepad1.a || gamepad2.a;
            if (aPressed && !a_button_previously_pressed) {
                flywheelOn = !flywheelOn;
            }
            a_button_previously_pressed = aPressed;

            if (flywheelOn) {
                // Convert RPM -> ticks/sec for setVelocity()
                double targetTicksPerSec = (TARGET_FLYWHEEL_RPM / 60.0) * ENCODER_COUNTS_PER_REV;
                shooterR.setVelocity(targetTicksPerSec);
            } else {
                shooterR.setVelocity(0);
            }

            //----------------//
            // TELEMETRY
            //----------------//
            double actualTicksPerSec = shooterR.getVelocity();
            double actualRPM = (actualTicksPerSec / ENCODER_COUNTS_PER_REV) * 60.0;

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("--- Field Centric ---", "");
            telemetry.addData("Heading (deg)", orientation.getYaw(AngleUnit.DEGREES));

            telemetry.addData("--- Shooter ---", "");
            telemetry.addData("Flywheel", flywheelOn ? "ON" : "OFF");
            telemetry.addData("Target RPM", TARGET_FLYWHEEL_RPM);
            telemetry.addData("Target ticks/s",
                    (TARGET_FLYWHEEL_RPM / 60.0) * ENCODER_COUNTS_PER_REV);
            telemetry.addData("Actual RPM", "%.1f", actualRPM);
            telemetry.addData("Actual ticks/s", "%.1f", actualTicksPerSec);
            telemetry.addData("Enc Pos", shooterR.getCurrentPosition());
            telemetry.addData("PIDF", "P=%.1f I=%.1f D=%.1f F=%.3f",
                    shooterP, shooterI, shooterD, shooterF);
            telemetry.update();
        }
    }
}
