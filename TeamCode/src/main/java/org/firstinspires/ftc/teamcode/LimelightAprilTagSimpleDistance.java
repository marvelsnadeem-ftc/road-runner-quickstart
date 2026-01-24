package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

//Using Pinpoint RoadRunner
@TeleOp
public class LimelightAprilTagSimpleDistance extends OpMode {

    private Limelight3A limelight;
    private MecanumDrive drive;
    private double distance;

    @Override
    public void init() {
        // --- Road Runner drive (your class) ---
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        // --- Limelight ---
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {
        // 1) Drive update FIRST so pose/heading is current
        drive.updatePoseEstimate();

        // 2) Read robot heading from Pinpoint via RR pose
        // Road Runner heading is usually in radians.
        double headingRad = drive.localizer.getPose().heading.toDouble();     // if heading is Rotation2d
        // If your pose uses double heading directly, use: double headingRad = drive.pose.heading;

        double headingDeg = Math.toDegrees(headingRad);
        headingDeg = wrapDeg180(headingDeg);

        // 3) Feed yaw to Limelight (expects degrees)
        limelight.updateRobotOrientation(headingDeg);

//        // 4) OPTIONAL: basic robot-centric driving (keep or remove)
//        // Stick mapping: forward = -left_stick_y, strafe = -left_stick_x, turn = -right_stick_x
//        double forward = -gamepad1.left_stick_y;
//        double strafe  = -gamepad1.left_stick_x;
//        double turn    = -gamepad1.right_stick_x;
//
//        drive.setDrivePowers(new PoseVelocity2d(
//                new Vector2d(forward, strafe),
//                turn
//        ));

        // 5) Read Limelight result
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D botPose = result.getBotpose_MT2();
            distance = getDistanceFromTag(result.getTa());
            telemetry.addData("Target Found", true);
            telemetry.addData("calculated distance(cm)", distance);
            telemetry.addData("calculated distance(in)", distance/2.54);
            telemetry.addData("Tx", result.getTx());
            telemetry.addData("Ty", result.getTy());
            telemetry.addData("Ta", result.getTa());
            telemetry.addData("BotPose_MT2", botPose.toString());
        } else {
            telemetry.addData("Target Found", false);
        }

        telemetry.addData("RR Heading (deg)", headingDeg);
        telemetry.addData("RR Pose", drive.localizer.getPose());
        telemetry.update();
    }

    private static double wrapDeg180(double deg) {
        while (deg <= -180.0) deg += 360.0;
        while (deg >   180.0) deg -= 360.0;
        return deg;
    }

    public static double getDistanceFromTag(double ta){
//        double scale = 25948.4979;
//        double distance = Math.sqrt(scale / ta);
//        return distance;

        double a = 25948.4979;//29662.36;
        double bAbs = 1.951401;

        double distance = Math.pow(a / ta, 1.0 / bAbs);
        return distance;
    }


  //Uses imu

    //    Limelight3A limelight;
//
//    @Override
//    public void init() {
//        limelight = hardwareMap.get(Limelight3A.class, "limelight");
//        limelight.pipelineSwitch(0);
//
//    }
//
//    @Override
//    public void start(){
//        limelight.start(); // Start streaming data
//    }
//
//    @Override
//    public void loop() {
//        YawPitchRollAngles orientation = bench.getOrientation();
//        limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));
//
//        LLResult result = limelight.getLatestResult();
//        if (result != null && result.isValid()) {
//            Pose3D botPose = result.getBotpose_MT2();
//
//            telemetry.addData("Target Found", true);
//            telemetry.addData("Tx", result.getTx());
//            telemetry.addData("Ty", result.getTy());
//            telemetry.addData("Ta", result.getTa());
//            telemetry.addData("BotPose", botPose.toString());
//
//        } else {
//            telemetry.addData("Target Found", false);
//        }
//        telemetry.update();
//    }
}
