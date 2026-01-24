package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="Flywheel_MaxVelocity_Test_AS")
public class Flywheel_MaxVelocity_Test_AS extends LinearOpMode {

    private DcMotorEx flywheel;

    @Override
    public void runOpMode() {
        flywheel = hardwareMap.get(DcMotorEx.class, "shooterR");

        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setPower(0);

        telemetry.addLine("Press A to run full power ~2s and measure max getVelocity()");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {


//            telemetry.addLine("Rotate shaft by hand exactly 1 revolution (tape mark to tape mark)");
//            telemetry.addData("Counts", flywheel.getCurrentPosition());
//            telemetry.update();

            // if (gamepad1.a) {
            //     // Spin up at full power
            //     flywheel.setPower(1.0);

            //     ElapsedTime t = new ElapsedTime();
            //     double maxTicksPerSec = 0;

            //     // Sample velocity for ~2 seconds
            //     while (opModeIsActive() && t.seconds() < 5.0) {
            //         double v = flywheel.getVelocity();     // ticks/sec
            //         if (v > maxTicksPerSec) maxTicksPerSec = v;

            //         telemetry.addData("Instant vel (ticks/s)", v);
            //         telemetry.addData("Max vel (ticks/s)", maxTicksPerSec);
            //         telemetry.update();
            //     }

            //     // Stop motor
            //     flywheel.setPower(0);

            //     // Compute kF
            //     double kF = 32767.0 / maxTicksPerSec;

            //     telemetry.addLine("==== RESULT ====");
            //     telemetry.addData("Measured max ticks/s", maxTicksPerSec);
            //     telemetry.addData("Computed kF", kF);
            //     telemetry.addLine("Release A to re-run test.");
            //     telemetry.update();

            //     // Simple latch so it doesn't immediately re-run while A is held
            //     while (opModeIsActive() && gamepad1.a) {
            //         idle();
            //     }
            // }

            // idle();
        }
    }
}
