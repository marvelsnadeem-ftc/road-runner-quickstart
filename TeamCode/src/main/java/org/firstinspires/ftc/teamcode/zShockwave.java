package org.firstinspires.ftc.teamcode;

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

@TeleOp(name = "zShockwave_backup")
public class zShockwave extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    // Drive Motors
    private DcMotor fl = null;
    private DcMotor fr = null;
    private DcMotor bl = null;
    private DcMotor br = null;

    // Mechanism Motors
    private DcMotor intake = null;
    //private DcMotorEx shooterL = null;
    private DcMotorEx shooterR = null;

    // Servos
    private Servo hold = null;

    // IMU
    private IMU imu;

    // --- State Variables for Toggles ---
    boolean flywheelOn = false;
    boolean a_button_previously_pressed = false;

    // --- CONTROL CONSTANTS ---
    static final double BANG_BANG_TARGET_VELOCITY = 2000.0;
    static final double FLYWHEEL_FULL_POWER = -0.7;
    static final double INTAKE_POWER = 1.0;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        // --- HARDWARE MAPPING ---
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");

        intake   = hardwareMap.get(DcMotor.class, "intake");
        //shooterL = hardwareMap.get(DcMotorEx.class, "shooterL");
        shooterR = hardwareMap.get(DcMotorEx.class, "shooterR");

        // ✅ Correct IMU orientation: Label UP, USB BACK
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
        //shooterL.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.FORWARD);

        // --- MOTOR BEHAVIOR ---
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //shooterL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // --- ENCODER SETUP ---
        shooterR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //open door
        hold.setPosition(0.9);

        telemetry.addData("Status", "Initialized. Ready to run!");
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

            // Press OPTIONS to reset field orientation
            if (gamepad1.options) {
                imu.resetYaw();
            }

            //----------------//
            // INTAKE CONTROL (Both Gamepads)
            //----------------//
            boolean intakeForwardTrigger = gamepad1.left_trigger > 0.1 || gamepad2.left_trigger > 0.1;
            boolean intakeReverseTrigger = gamepad1.right_trigger > 0.1 || gamepad2.right_trigger > 0.1;

//            if (intakeReverseTrigger) {
//                intake.setPower(-INTAKE_POWER);
//            } else if (intakeForwardTrigger) {
//                intake.setPower(INTAKE_POWER);
//            } else {
//                intake.setPower(0);
//            }

            if (gamepad1.x) {
                //open door
                hold.setPosition(0.9);
                intake.setPower(-INTAKE_POWER);
            } else {
                //close door
                hold.setPosition(0.4);
                if (intakeReverseTrigger) {
                    intake.setPower(-INTAKE_POWER);
                } else if (intakeForwardTrigger) {
                    intake.setPower(INTAKE_POWER);
                } else {
                    intake.setPower(0);
                }
            }

            //----------------//
            // FLYWHEEL SHOOTER (Both Gamepads)
            //----------------//
            boolean aPressed = gamepad1.a || gamepad2.a;
            if (aPressed && !a_button_previously_pressed) {
                flywheelOn = !flywheelOn;
            }
            a_button_previously_pressed = aPressed;

            if (flywheelOn) {
                double currentVelocity = -shooterR.getVelocity();

                if (currentVelocity < BANG_BANG_TARGET_VELOCITY) {
                    //shooterL.setPower(FLYWHEEL_FULL_POWER);
                    shooterR.setPower(FLYWHEEL_FULL_POWER);
                } else {
                    //shooterL.setPower(0);
                    shooterR.setPower(0);
                }
            } else {
                //shooterL.setPower(0);
                shooterR.setPower(0);
            }

            //----------------//
            // TELEMETRY      //
            //----------------//
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("--- Field Centric ---", "");
            telemetry.addData("Heading (deg)", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.addData("--- Shooter ---", "");
            telemetry.addData("Flywheel", flywheelOn ? "ON" : "OFF");
            telemetry.addData("Target Vel", BANG_BANG_TARGET_VELOCITY);
            telemetry.addData("Actual Vel", "%.2f", shooterR.getVelocity());
            telemetry.update();
        }
    }
}
