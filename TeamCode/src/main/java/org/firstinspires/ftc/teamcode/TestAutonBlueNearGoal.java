package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.autos.FieldCoordinates;
import org.firstinspires.ftc.teamcode.autos.TileMoveHelper;
import org.firstinspires.ftc.teamcode.utils.HelperUtils;

@Autonomous(name = "Test Shockwave Auton Blue Near", group = "Auto")
public class TestAutonBlueNearGoal extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();

    // Same timing constants as Red Near
    private static final double INTAKE_FORWARD_SEC  = 1.0;
    private static final double INTAKE_REVERSE_SEC  = 0.20;

    @Override
    public void runOpMode() throws InterruptedException {

        // Seed pose; Pinpoint / RR localizer will give live pose
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        // Shooter runs continuously with bang-bang (same pattern as your Red Near)
        Shooter nShooter = new Shooter(hardwareMap, 2800);
        Shooter fShooter = new Shooter(hardwareMap, 3500);
        Intake  intake   = new Intake(hardwareMap);

        // ===== ADDED: Limelight init (minimum) =====
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        int pipelineSelect = 0; // 0 is blue, 1 is for red
        limelight.pipelineSwitch(pipelineSelect); //Red
        limelight.start();
        // ==========================================

        HelperUtils utils = new HelperUtils(runtime, telemetry, INTAKE_FORWARD_SEC, INTAKE_REVERSE_SEC);
        utils.setLimelightSearchOmega(1.2); // -ve for Red, +ve for Blue // // set this value only once and only for Auto.
        utils.setLimelight(limelight);
        utils.setLimelightPipeline(pipelineSelect);


        // Show live pose during INIT
        while (!isStopRequested() && !opModeIsActive()) {
            Pose2d p = drive.localizer.getPose();
            telemetry.addLine("INIT Pose (from Pinpoint / RR):");
            telemetry.addData("X", p.position.x);
            telemetry.addData("Y", p.position.y);
            telemetry.addData("Heading (deg)", Math.toDegrees(p.heading.toDouble()));
            telemetry.update();
        }

        // Use same sign config you used successfully
        TileMoveHelper.X_SIGN = -1.0;
        TileMoveHelper.Y_SIGN = -1.0;

        Pose2d shootingPoseNear;

        waitForStart();
        if (isStopRequested()) return;

        runtime.reset();

        // Bursts (fresh each time)
        Action burstPreload = utils.buildIntakeBurst(intake, 3);
        Action burstA5      = utils.buildIntakeBurst(intake, 3);
        Action burstA4      = utils.buildIntakeBurst(intake, 3);
        Action burstA3      = utils.buildIntakeBurst(intake, 3);
        Action burstA2      = utils.buildIntakeBurst(intake, 3);



        // ============================================================
        // PART 1 (BLUE NEW): From B6, drive to C4, turn left toward A6, shoot preloads
        // ============================================================

        // Start pose comes from Pinpoint / RR localizer (robot placed at B6)
        Pose2d startPose = drive.localizer.getPose();

        // Optional (ONLY if your init pose is not already correct at B6):
        // drive.localizer.setPose(new Pose2d(-60, -36, Math.toRadians(180))); // adjust heading if needed

        double tile = FieldCoordinates.TILE;

        // B6 -> C4 is: +2 tiles in X (Row6 -> Row4), +1 tile in Y (B -> C)
        double targetX_C4 = startPose.position.x + 2.3 * tile;
        double targetY_C4 = startPose.position.y + 1.0 * tile;

        // Same aiming math as your working code: A6 relative to C4 is 2 tiles up + 2 tiles left => 45°
        double aimOffsetRad = Math.atan2(2.0, 2.0); // 45 deg

////        double travelTangentToC4 = Math.atan2(
////                targetY_C4 - startPose.position.y,
////                targetX_C4 - startPose.position.x
////        );
////
////        // Keep heading the same while translating to C4, then do the same +45° turn
////        Action moveToC4TurnAndShootPose = drive.actionBuilder(startPose)
////                .splineToLinearHeading(
////                        new Pose2d(targetX_C4, targetY_C4, startPose.heading.toDouble()),
////                        travelTangentToC4
////                )
////                .turn(+aimOffsetRad)     // CCW (left) 45°
////                .build();
//
//        // 1) Move to C4 (no vision blending here; keep path accurate)
//        Action moveToC4TurnAndShootPose = drive.actionBuilder(startPose)
//                .strafeTo(new Vector2d(targetX_C4, targetY_C4))
//                .turn(aimOffsetRad)
//                .build();
//
//        Action moveToC4TurnAndShootPoseWithVision = utils.driveWithLimelightTx(moveToC4TurnAndShootPose, drive);
//        Actions.runBlocking(
//                utils.withPoseTelemetry(moveToC4TurnAndShootPoseWithVision, drive, nShooter, false)
//        );

        Action moveToC4 = drive.actionBuilder(startPose)
                .strafeTo(new Vector2d(targetX_C4, targetY_C4))
                .build();

        Actions.runBlocking(utils.withPoseTelemetry(moveToC4, drive, nShooter, false));

        // Vision search + lock (blue pipeline selected beforehand)
        Actions.runBlocking(utils.withPoseTelemetry(
                utils.alignToTagTx(drive, 1.5),
                drive, nShooter, false
        ));


        // Shoot 3 preloads (intake burst only; shooter already running)
//        Actions.runBlocking(utils.withPoseTelemetry(
//                burstPreload,
//                drive, nShooter, true
//        ));

        // Record shooting pose AFTER the turn (this is what you return to later)
        shootingPoseNear = drive.localizer.getPose();

        telemetry.addLine("Blue shooting pose recorded (C4-ish pointing A6)");
        telemetry.addData("Shoot X", shootingPoseNear.position.x);
        telemetry.addData("Shoot Y", shootingPoseNear.position.y);
        telemetry.addData("Shoot H (deg)", Math.toDegrees(shootingPoseNear.heading.toDouble()));
        telemetry.update();

        // ============================================================
        // PART 2 (BLUE): from shootingPoseNear -> A5|B5 seam midpoint
        // Turn early by the time we reach B5, then strafe into the seam.
        // Uses RELATIVE tile deltas from shootingPoseNear.
        // Convention for THIS Blue start:
        //   upfield (toward row 6) = X-
        //   toward A (left)        = Y-
        // ============================================================

        // Deltas from C4 -> Row5 and toward A5/B5 seam
        double rowsUpToRow5      = 2.0;   // C4 -> C5 lane (one row up)
        double colsTowardB5      = 0.5;   // C -> B (one column toward A)
        double colsTowardA5B5Mid  = 1;   // C -> seam between B and A

        // Convert to inches with your Blue sign convention:
        // upfield is X-, toward A is Y-
        double dxRow5   = -rowsUpToRow5     * tile;   // X-
        double dyB5     = -colsTowardB5     * tile;   // Y-
        double dyA5B5Seam = -colsTowardA5B5Mid * tile;   // Y-

        // Intermediate point (B5-ish) where we want to be "already turned"
        double xB5 = shootingPoseNear.position.x + dxRow5;
        double yB5 = shootingPoseNear.position.y + dyB5;

        // Final pickup target: seam midpoint between A5 and B5 (same row5 X)
        double xA5Seam = shootingPoseNear.position.x + dxRow5;
        double yA5Seam = shootingPoseNear.position.y + dyA5B5Seam;

        // Intake should face toward A (left) = -Y direction => heading -90°
        // (This matches the “turn early” intent: intake is square before seam)
        double intakeHeadingTowardA5 = Math.toRadians(-90.0);

        // Tangent into the midpoint (direction of travel start->B5 point)
        double tangentToB5 = Math.atan2(yB5 - shootingPoseNear.position.y, xB5 - shootingPoseNear.position.x);

        // Build ONE trajectory:
        //  1) spline to B5-ish point while rotating to intakeHeadingTowardA (turn happens early)
        //  2) strafe straight into the seam midpoint
        Pose2d poseAtB5 = new Pose2d(xB5, yB5, intakeHeadingTowardA5);

        Action toA5B5SeamTurnEarly = drive.actionBuilder(shootingPoseNear)
                .splineToLinearHeading(poseAtB5, tangentToB5)
                .strafeTo(new Vector2d(xA5Seam, yA5Seam))
                .build();

        // Intake only while driving (stops exactly when the drive action ends)
        Action toA5B5SeamWithIntake = utils.driveWithIntake(toA5B5SeamTurnEarly, intake, 1.0);
        Actions.runBlocking(utils.withPoseTelemetry(toA5B5SeamWithIntake, drive, nShooter, false));

        // Back to shooting pose towards A6, then shoot the 3 pickup balls
        Pose2d poseAtA5 = drive.localizer.getPose();

        // 0) CLEAR the seam first (move away from A wall / seam so you don't clip A4|B4 balls)
        // Go back to the B5-ish intermediate point you already used
        Action clearA5Seam = drive.actionBuilder(poseAtA5)
                        .strafeTo(new Vector2d(xB5, yB5))
                        .build();

        Actions.runBlocking(utils.withPoseTelemetry(clearA5Seam, drive, nShooter, false));

        // 1) Now return from the cleared point to shootingPoseNear
        Pose2d poseAfterA4B4Clear = drive.localizer.getPose();
        Action poseBackToShootingPoseFromA5 = utils.GoToPose(poseAfterA4B4Clear, drive, shootingPoseNear);
        Action poseBackToShootingPoseFromA5WithVision = utils.driveWithLimelightTx(poseBackToShootingPoseFromA5, drive);

        Actions.runBlocking(utils.withPoseTelemetry(poseBackToShootingPoseFromA5WithVision, drive, nShooter, false));


//        // Back to shooting pose towards A6, then shoot the 3 pickup balls
//        Pose2d poseAtA5 = drive.localizer.getPose();
//        Action poseBackToShootingPoseFromA5 = utils.GoToPose(poseAtA5, drive, shootingPoseNear); //GoToPose(fromPose, drive, toPose) Go to from currentpose to a new topose
//        Action poseBackToShootingPoseFromA5WithVision = utils.driveWithLimelightTx(poseBackToShootingPoseFromA5, drive);
//        Actions.runBlocking(utils.withPoseTelemetry(poseBackToShootingPoseFromA5WithVision, drive, nShooter, false));

        // FIX: Re-seed pose estimate to the known-good shootingPoseNear to prevent drift compounding
        //shootingPoseNear = drive.localizer.getPose();

//        Actions.runBlocking(utils.withPoseTelemetry(
//                new SequentialAction(
//                        burstA5
//                ),
//                drive, nShooter, true
//        ));


        // ============================================================
        // PART 3 (BLUE): from shootingPoseNear -> A4|B4 seam midpoint
        // Turn early by the time we reach B4, then strafe into the seam.
        // Uses RELATIVE tile deltas from shootingPoseNear.
        // Convention for THIS Blue start:
        //   upfield (toward row 4) = X-
        //   toward A (left)        = Y-
        // ============================================================

        // Deltas from C4 and toward A4/B4 seam
        double rowsUpToRow4      = 1.0;   // C4 -> C4 lane (same lane)
        double colsTowardB4      = 0.5;   // C -> B (one column toward A)
        double colsTowardB4A4Mid  = 1.0;   // C -> seam between B and A

        // Convert to inches with your Blue sign convention:
        // upfield is X-, toward A is Y-
        double dxRow4   = -rowsUpToRow4     * tile;   // X-
        double dyB4     = -colsTowardB4     * tile;   // Y-
        double dyA4B4Seam = -colsTowardB4A4Mid * tile;   // Y-

        // Intermediate point (B4-ish) where we want to be "already turned"
        double xB4 = shootingPoseNear.position.x + dxRow4;
        double yB4 = shootingPoseNear.position.y + dyB4;

        // Final pickup target: seam midpoint between A5 and B5 (same row5 X)
        double x4Seam = shootingPoseNear.position.x + dxRow4;
        double y4Seam = shootingPoseNear.position.y + dyA4B4Seam;

        // Intake should face toward A (left) = -Y direction => heading -90°
        // (This matches the “turn early” intent: intake is square before seam)
        double intakeHeadingTowardA4 = Math.toRadians(-90.0);

        // Tangent into the midpoint (direction of travel start->B5 point)
        double tangentToB4 = Math.atan2(yB4 - shootingPoseNear.position.y, xB4 - shootingPoseNear.position.x);

        // Build ONE trajectory:
        //  1) spline to B5-ish point while rotating to intakeHeadingTowardA (turn happens early)
        //  2) strafe straight into the seam midpoint
        Pose2d poseAtB4 = new Pose2d(xB4, yB4, intakeHeadingTowardA4);

        Action toA4B4SeamTurnEarly = drive.actionBuilder(shootingPoseNear)
                .splineToLinearHeading(poseAtB4, tangentToB4)
                .strafeTo(new Vector2d(x4Seam, y4Seam))
                .build();

        // Intake only while driving (stops exactly when the drive action ends)
        Action toA4B4SeamWithIntake = utils.driveWithIntake(toA4B4SeamTurnEarly, intake, 1.0);
        Actions.runBlocking(utils.withPoseTelemetry(toA4B4SeamWithIntake, drive, nShooter, false));

        // Back to shooting pose towards A6, then shoot the 3 pickup balls
        Pose2d poseAtA4 = drive.localizer.getPose();
        Action poseBackToShootingPoseFromA4 = utils.GoToPose(poseAtA4, drive, shootingPoseNear); //GoToPose(fromPose, drive, toPose) Go to from currentpose to a new topose
        Action poseBackToShootingPoseFromA4WithVision = utils.driveWithLimelightTx(poseBackToShootingPoseFromA4, drive);
        Actions.runBlocking(utils.withPoseTelemetry(poseBackToShootingPoseFromA4WithVision, drive, nShooter, false));

        // FIX: Re-seed pose estimate to the known-good shootingPoseNear to prevent drift compounding
        //shootingPoseNear = drive.localizer.getPose();

//        Actions.runBlocking(utils.withPoseTelemetry(
//                new SequentialAction(
//                        burstA4
//                ),
//                drive, nShooter, true
//        ));

        // ============================================================
        // PART 4 (BLUE): from shootingPoseNear -> A3|B3 seam midpoint
        // Turn early by the time we reach B3, then strafe into the seam.
        // Uses RELATIVE tile deltas from shootingPoseNear.
        // Convention for THIS Blue start:
        //   downfield (toward row 3) = X+
        //   toward A (left)        = Y-
        // ============================================================

        // Deltas from C3 and toward A3/B3 seam
        double rowsDownToRow3      = 0.0;   // C4 -> C3 lane (one row down)
        double colsTowardB3      = 0.5;   // C -> B (one column toward A)
        double colsTowardB3A3Mid  = 1.0;   // C -> seam between B and A

        // Convert to inches with your Blue sign convention:
        // downfield is X+, toward A is Y-
        double dxRow3   = rowsDownToRow3     * tile;   // X+
        double dyB3     = -colsTowardB3     * tile;   // Y-
        double dyA3B3Seam = -colsTowardB3A3Mid * tile;   // Y-

        // Intermediate point (B3-ish) where we want to be "already turned"
        double xB3 = shootingPoseNear.position.x + dxRow3;
        double yB3 = shootingPoseNear.position.y + dyB3;

        // Final pickup target: seam midpoint between A5 and B5 (same row5 X)
        double x3Seam = shootingPoseNear.position.x + dxRow3;
        double y3Seam = shootingPoseNear.position.y + dyA3B3Seam;

        // Intake should face toward A (left) = -Y direction => heading -90°
        // (This matches the “turn early” intent: intake is square before seam)
        double intakeHeadingTowardA3 = Math.toRadians(-90.0);

        // Tangent into the midpoint (direction of travel start->B5 point)
        double tangentToB3 = Math.atan2(yB3 - shootingPoseNear.position.y, xB3 - shootingPoseNear.position.x);

        // Build ONE trajectory:
        //  1) spline to B5-ish point while rotating to intakeHeadingTowardA (turn happens early)
        //  2) strafe straight into the seam midpoint
        Pose2d poseAtB3 = new Pose2d(xB3, yB3, intakeHeadingTowardA3);

        Action toA3B3SeamTurnEarly = drive.actionBuilder(shootingPoseNear)
                .splineToLinearHeading(poseAtB3, tangentToB3)
                .strafeTo(new Vector2d(x3Seam, y3Seam))
                .build();

        // Intake only while driving (stops exactly when the drive action ends)
        Action toA3B3SeamWithIntake = utils.driveWithIntake(toA3B3SeamTurnEarly, intake, 1.0);
        Actions.runBlocking(utils.withPoseTelemetry(toA3B3SeamWithIntake, drive, nShooter, false));

        // Back to shooting pose towards A6, then shoot the 3 pickup balls
        Pose2d poseAtA3 = drive.localizer.getPose();
        Action poseBackToShootingPoseFromA3 = utils.GoToPose(poseAtA3, drive, shootingPoseNear); //GoToPose(fromPose, drive, toPose) Go to from currentpose to a new topose
        Action poseBackToShootingPoseFromA3WithVision = utils.driveWithLimelightTx(poseBackToShootingPoseFromA3, drive);
        Actions.runBlocking(utils.withPoseTelemetry(poseBackToShootingPoseFromA3WithVision, drive, nShooter, false));


//        Actions.runBlocking(utils.withPoseTelemetry(
//                new SequentialAction(
//                        burstA3
//                ),
//                drive, nShooter, true
//        ));

        // ============================================================
        // PART 5 (BLUE): from shootingPoseNear -> A2|B2 seam midpoint
        // Turn early by the time we reach B2, then strafe into the seam.
        // Uses RELATIVE tile deltas from shootingPoseNear.
        // Convention for THIS Blue start:
        //   downfield (toward row 2) = X+
        //   toward A (left)        = Y-
        // ============================================================

        // Deltas from C2 and toward A2/B2 seam
        double rowsDownToRow2      = 0.8;   // C4 -> C2 lane (one row down)
        double colsTowardB2      = 0.5;   // C -> B (one column toward A)
        double colsTowardB2A2Mid  = 1.0;   // C -> seam between B and A

        // Convert to inches with your Blue sign convention:
        // downfield is X+, toward A is Y-
        double dxRow2   = rowsDownToRow2     * tile;   // X+
        double dyB2     = -colsTowardB2     * tile;   // Y-
        double dyA2B2Seam = -colsTowardB2A2Mid * tile;   // Y-

        // Intermediate point (B2-ish) where we want to be "already turned"
        double xB2 = shootingPoseNear.position.x + dxRow2;
        double yB2 = shootingPoseNear.position.y + dyB2;

        // Final pickup target: seam midpoint between A5 and B5 (same row5 X)
        double x2Seam = shootingPoseNear.position.x + dxRow2;
        double y2Seam = shootingPoseNear.position.y + dyA2B2Seam;

        // Intake should face toward A (left) = -Y direction => heading -90°
        // (This matches the “turn early” intent: intake is square before seam)
        double intakeHeadingTowardA2 = Math.toRadians(-90.0);

        // Tangent into the midpoint (direction of travel start->B5 point)
        double tangentToB2 = Math.atan2(yB2 - shootingPoseNear.position.y, xB2 - shootingPoseNear.position.x);

        // Build ONE trajectory:
        //  1) spline to B5-ish point while rotating to intakeHeadingTowardA (turn happens early)
        //  2) strafe straight into the seam midpoint
        Pose2d poseAtB2 = new Pose2d(xB2, yB2, intakeHeadingTowardA2);

        Action toA2B2SeamTurnEarly = drive.actionBuilder(shootingPoseNear)
                .splineToLinearHeading(poseAtB2, tangentToB2)
                .strafeTo(new Vector2d(x2Seam, y2Seam))
                .build();

        // Intake only while driving (stops exactly when the drive action ends)
        Action toA2B2SeamWithIntake = utils.driveWithIntake(toA2B2SeamTurnEarly, intake, 1.0);
        Actions.runBlocking(utils.withPoseTelemetry(toA2B2SeamWithIntake, drive, nShooter, false));

        // Back to shooting pose towards A6, then shoot the 3 pickup balls
        Pose2d poseAtA2 = drive.localizer.getPose();
        Pose2d shootingPoseFar = new Pose2d(110.7995, 25, Math.toRadians(25));
        Action poseBackToShootingPoseFromA2 = utils.GoToPose(poseAtA2, drive, shootingPoseFar); //GoToPose(fromPose, drive, toPose) Go to from currentpose to a new topose
        Action poseBackToShootingPoseFromA2WithVision = utils.driveWithLimelightTx(poseBackToShootingPoseFromA2, drive);
        Actions.runBlocking(utils.withPoseTelemetry(poseBackToShootingPoseFromA2WithVision, drive, nShooter, false));

//        Actions.runBlocking(utils.withPoseTelemetry(
//                new SequentialAction(
//                        burstA2
//                ),
//                drive, fShooter, true
//        ));


        // Optional: record for telemetry/debug
        shootingPoseFar = drive.localizer.getPose();

    }

}






//package org.firstinspires.ftc.teamcode;
//
//import androidx.annotation.NonNull;
//
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.SequentialAction;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.autos.FieldCoordinates;
//import org.firstinspires.ftc.teamcode.autos.TileMoveHelper;
//
//@Autonomous(name = "Test Shockwave Auton Blue Near", group = "Auto")
//public class TestAutonBlueNear extends LinearOpMode {
//
//    private final ElapsedTime runtime = new ElapsedTime();
//    // === TIMING CONSTANTS (tune these on field) ===
//    private static final double INTAKE_FORWARD_SEC  = 1.0;   // was 2.0
//    private static final double INTAKE_REVERSE_SEC  = 0.20;  // was 0.2
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
//
//        // Single shooter instance (continuous run)
//        Shooter shooter = new Shooter(hardwareMap, 2800);
//
//        Intake intake = new Intake(hardwareMap);
//
//        // Shooter removed from ShootAndIntakeAction (intake-only actions now)
////        ShootAndIntakeAction intakeShoot = new ShootAndIntakeAction(intake, 2.0, 1.0);
////        ShootAndIntakeAction intakeShootReverse = new ShootAndIntakeAction(intake, 0.2, -1.0);
//
//        // Show live pose on INIT so you can sanity check Pinpoint signs.
//        while (!isStopRequested() && !opModeIsActive()) {
//            Pose2d p = drive.localizer.getPose();
//            telemetry.addLine("INIT Pose (from Pinpoint / RR):");
//            telemetry.addData("X", p.position.x);
//            telemetry.addData("Y", p.position.y);
//            telemetry.addData("Heading (deg)", Math.toDegrees(p.heading.toDouble()));
//            telemetry.update();
//        }
//
//        TileMoveHelper.X_SIGN = -1.0;
//        TileMoveHelper.Y_SIGN = -1.0;
//
//        Pose2d shootingPoseNear;
//
//        waitForStart();
//        if (isStopRequested()) return;
//        runtime.reset();
//
//        // Build bursts as needed (fresh action each time)
//        Action burstPreload = buildIntakeBurst(intake, 3);
//        Action burstA3      = buildIntakeBurst(intake, 3);
//        Action burstA2      = buildIntakeBurst(intake, 3);
//        Action burstA1      = buildIntakeBurst(intake, 3);
//
//        // PART 1 Go and Shoot
//        Pose2d startPose = drive.localizer.getPose(); // flywheel facing C6 (up)
//
//        double deltaFieldForward = 2.75 * TileMoveHelper.TILE;
//        double deltaRobotX       = TileMoveHelper.X_SIGN * deltaFieldForward;
//        double targetX           = startPose.position.x + deltaRobotX;
//
//        Action moveRows3TurnAndShootPose = drive.actionBuilder(startPose)
//                .lineToX(targetX)
//                .turn(Math.toRadians(35))
//                .build();
//
//        // Moving => trapdoor must be closed
//        Actions.runBlocking(withPoseTelemetry(moveRows3TurnAndShootPose, drive, shooter, false));
//
//        // Preload shoot (was intakeReverse1/2/3 + intakeBall1/2/3)
//        Actions.runBlocking(withPoseTelemetry(
//                burstPreload,
//                drive, shooter, true
//        ));
//
//        shootingPoseNear = drive.localizer.getPose();
//        telemetry.addLine("Shooting pose recorded (C4-ish)");
//        telemetry.addData("Shoot X", shootingPoseNear.position.x);
//        telemetry.addData("Shoot Y", shootingPoseNear.position.y);
//        telemetry.addData("Shoot H (deg)", Math.toDegrees(shootingPoseNear.heading.toDouble()));
//        telemetry.update();
//
//        // PART 2: rotate intake, move to A3 while intaking
//        Action rotateIntakeToA3 = drive.actionBuilder(shootingPoseNear)
//                .turn(Math.toRadians(-125))
//                .build();
//
//        Actions.runBlocking(withPoseTelemetry(rotateIntakeToA3, drive, shooter, false));
//
//        Pose2d poseAfterTurn = drive.localizer.getPose();
//        double tile = FieldCoordinates.TILE;
//
//        double deltaRows = 1.5 * tile;
//        double colsLeft = 1.20 * tile;
//
//        double flywheelHeading = poseAfterTurn.heading.toDouble();
//        double intakeHeading   = flywheelHeading + Math.PI;
//
//        double forwardX = poseAfterTurn.position.x + TileMoveHelper.X_SIGN * deltaRows * Math.cos(intakeHeading);
//        double forwardY = poseAfterTurn.position.y + TileMoveHelper.Y_SIGN * colsLeft * Math.sin(intakeHeading);
//
//        Action strafetoA3 = drive.actionBuilder(poseAfterTurn)
//                .strafeTo(new Vector2d(forwardX, forwardY))
//                .build();
//
//        Action part2DriveWithIntake = driveWithIntake(strafetoA3, intake, 1.0);
//        // Moving => trapdoor closed
//        Actions.runBlocking(withPoseTelemetry(part2DriveWithIntake, drive, shooter, false));
//
//        // Back to shootingPose, then shoot sequence (trapdoor allowed only during shooting sequence)
//        Pose2d poseAtA3 = drive.localizer.getPose();
//        Action poseBackToShootingPoseFromA2 = ShootingPose(shootingPoseNear, drive, poseAtA3);
//
//        Actions.runBlocking(withPoseTelemetry(poseBackToShootingPoseFromA2, drive, shooter, false));
//
//        // A3 shoot
//        Actions.runBlocking(withPoseTelemetry(
//                burstA3,
//                drive, shooter, true
//        ));
//
//        // PART 3: Turn -45, spline to A2 while intaking, return and shoot
//        Action turnMinus45FromShoot = drive.actionBuilder(shootingPoseNear)
//                .turn(Math.toRadians(-45))
//                .build();
//        Actions.runBlocking(withPoseTelemetry(turnMinus45FromShoot, drive, shooter, false));
//
//        Pose2d poseAtC3 = drive.localizer.getPose();
//
//        double deltaRowsToA2   = -1 * tile;
//        double deltaLeftToA2   =  1.2 * tile;
//
//        double deltaX_A2 = TileMoveHelper.X_SIGN * deltaRowsToA2;
//        double deltaY_A2 = TileMoveHelper.Y_SIGN * deltaLeftToA2;
//
//        double targetX_A2 = poseAtC3.position.x + deltaX_A2;
//        double targetY_A2 = poseAtC3.position.y + deltaY_A2;
//
//        double travelTangent_A2 = Math.atan2(
//                targetY_A2 - poseAtC3.position.y,
//                targetX_A2 - poseAtC3.position.x
//        );
//
//        double headingOffset_A2 = Math.toRadians(-38);
//        double finalHeading_A2  = travelTangent_A2 + headingOffset_A2;
//
//        Pose2d a2IntakePose = new Pose2d(
//                targetX_A2,
//                targetY_A2,
//                finalHeading_A2
//        );
//
//        Action splineToA2 = drive.actionBuilder(poseAtC3)
//                .splineToLinearHeading(
//                        a2IntakePose,
//                        travelTangent_A2
//                )
//                .build();
//
//        // Intake runs only while splineToA2 is executing
//        Action part3DriveWithIntake = driveWithIntake(splineToA2, intake, 1.0);
//        Actions.runBlocking(withPoseTelemetry(part3DriveWithIntake, drive, shooter, false));
//
//
//        Pose2d AfterA2 = drive.localizer.getPose();
//        Action poseBackToShootingPoseA2 = ShootingPose(shootingPoseNear, drive, AfterA2);
//
//        Actions.runBlocking(withPoseTelemetry(poseBackToShootingPoseA2, drive, shooter, false));
//
//
//        // A2 shoot
//        Actions.runBlocking(withPoseTelemetry(
//                burstA2,
//                drive, shooter, true
//        ));
//
//
//        // ===== PART 4: pickup A1, return, shoot =====
//
//// Read the starting pose for Part 4 (C4 shooting pose)
//        Pose2d part4Start = drive.localizer.getPose();
//
//// Row/column semantics (you already tuned these)
//        double rowsDown    = -1.0;    // C4 -> C3/C2-style drop
//        double deltaRowsToA1 = -0.8;  // extra down toward A1
//        double deltaLeftToA1 =  1.2;  // left toward A1
//
//// 1) Compute C2 row target in world coords from the Part4 start pose
//        double deltaFieldForwardDown = rowsDown * tile;
//        double deltaRobotXDown       = TileMoveHelper.X_SIGN * deltaFieldForwardDown;
//
//        double c2X = part4Start.position.x + deltaRobotXDown;
//        double c2Y = part4Start.position.y; // same lateral lane
//
//// 2) Compute A1 intake target from that C2 position
//        double deltaX_A1 = TileMoveHelper.X_SIGN * (deltaRowsToA1 * tile);
//        double deltaY_A1 = TileMoveHelper.Y_SIGN * (deltaLeftToA1 * tile);
//
//        double a1X = c2X + deltaX_A1;
//        double a1Y = c2Y + deltaY_A1;
//
//// 3) Direction of travel from C2 -> A1
//        double travelTangent_A1 = Math.atan2(
//                a1Y - c2Y,
//                a1X - c2X
//        );
//
//// 4) Final heading at A1 (intake orientation tweak)
//        double headingOffset_A1 = Math.toRadians(-35);  // your tuned value
//        double finalHeading_A1  = travelTangent_A1 + headingOffset_A1;
//
//        Pose2d a1IntakePose = new Pose2d(
//                a1X,
//                a1Y,
//                finalHeading_A1
//        );
//
//// 5) Build ONE smooth trajectory: turn -> move down 1 row -> spline to A1
//        Action part4Path = drive.actionBuilder(part4Start)
//                .turn(Math.toRadians(-45))      // STEP 1: turn from shooting pose
//                .lineToX(c2X)                   // STEP 2: drop one row to C2 lane
//                .splineToLinearHeading(         // STEP 3: curve into A1 intake pose
//                        a1IntakePose,
//                        travelTangent_A1
//                )
//                .build();
//
//// Intake runs only while this whole A1 path executes
//        Action part4DriveWithIntake = driveWithIntake(part4Path, intake, 1.0);
//
//// Single runBlocking for all A1 movement
//        Actions.runBlocking(withPoseTelemetry(part4DriveWithIntake, drive, shooter, false));
//
//        //Shooting Logic for A1
//        Pose2d poseAfterA1 = drive.localizer.getPose();
//        Action poseBackToShootingPoseA1 = ShootingPose(shootingPoseNear, drive, poseAfterA1);
//
//        Actions.runBlocking(withPoseTelemetry(poseBackToShootingPoseA1, drive, shooter, false));
//        // A1 shoot
//        Actions.runBlocking(withPoseTelemetry(
//                burstA1,
//                drive, shooter, true
//        ));
//
////        // PART 4: pickup A1, return, shoot
////        Action turnMinus45FromShoot2 = drive.actionBuilder(drive.localizer.getPose())
////                .turn(Math.toRadians(-45))
////                .build();
////        Actions.runBlocking(withPoseTelemetry(turnMinus45FromShoot2, drive, shooter, false));
////
////        poseAfterTurn = drive.localizer.getPose();
////
////        double rowsDown = -1.0;
////        double deltaFieldForwardDown = rowsDown * tile;
////
////        double deltaRobotXDown = TileMoveHelper.X_SIGN * deltaFieldForwardDown;
////        double targetX_C2 = poseAfterTurn.position.x + deltaRobotXDown;
////
////        Action moveToC2 = drive.actionBuilder(poseAfterTurn)
////                .lineToX(targetX_C2)
////                .build();
////
////        Actions.runBlocking(withPoseTelemetry(moveToC2, drive, shooter, false));
////
////        Pose2d poseAtC2 = drive.localizer.getPose();
////
////        double deltaRowsToA1   = -0.8 * tile;
////        double deltaLeftToA1   =  1.2 * tile;
////
////        double deltaX_A1 = TileMoveHelper.X_SIGN * deltaRowsToA1;
////        double deltaY_A1 = TileMoveHelper.Y_SIGN * deltaLeftToA1;
////
////        double targetX_A1 = poseAtC2.position.x + deltaX_A1;
////        double targetY_A1 = poseAtC2.position.y + deltaY_A1;
////
////        double travelTangent_A1 = Math.atan2(
////                targetY_A1 - poseAtC2.position.y,
////                targetX_A1 - poseAtC2.position.x
////        );
////
////        double headingOffset_A1 = Math.toRadians(-35);
////        double finalHeading_A1  = travelTangent_A1 + headingOffset_A1;
////
////        Pose2d a1IntakePose = new Pose2d(
////                targetX_A1,
////                targetY_A1,
////                finalHeading_A1
////        );
////
////        Action splineToA1 = drive.actionBuilder(poseAtC2)
////                .splineToLinearHeading(
////                        a1IntakePose,
////                        travelTangent_A1
////                )
////                .build();
////
////// Intake runs only while splineToA1 is executing
////        Action part4DriveWithIntake = driveWithIntake(splineToA1, intake, 1.0);
////        Actions.runBlocking(withPoseTelemetry(part4DriveWithIntake, drive, shooter, false));
//
////        //Shooting Logic for A1
////        Pose2d poseAfterA1 = drive.localizer.getPose();
////        Action poseBackToShootingPoseA1 = ShootingPose(shootingPoseNear, drive, poseAfterA1);
////
////        Actions.runBlocking(withPoseTelemetry(poseBackToShootingPoseA1, drive, shooter, false));
////        // A1 shoot
////        Actions.runBlocking(withPoseTelemetry(
////                burstA1,
////                drive, shooter, true
////        ));
//    }
//
//    private Action buildIntakeBurst(Intake intake, int numBalls) {
//        Action[] steps = new Action[numBalls * 2];
//        for (int i = 0; i < numBalls; i++) {
//            // reverse a bit
//            steps[2 * i]     = intake.intakeForwardForTime(INTAKE_REVERSE_SEC, -1.0);
//            // then feed forward
//            steps[2 * i + 1] = intake.intakeForwardForTime(INTAKE_FORWARD_SEC,  1.0);
//        }
//        return new SequentialAction(steps);
//    }
//
//
//    // Generic helper: from any currentPose, build an Action that splines back to shootingPose.
//    private Action ShootingPose(Pose2d shootingPose, MecanumDrive drive, Pose2d currentPose) {
//        if (shootingPose == null) {
//            throw new IllegalStateException("shootingPose is not set before calling buildSplineBackToShootingPose()");
//        }
//
//        double travelTangent = Math.atan2(
//                shootingPose.position.y - currentPose.position.y,
//                shootingPose.position.x - currentPose.position.x
//        );
//
//        return drive.actionBuilder(currentPose)
//                .splineToLinearHeading(shootingPose, travelTangent)
//                .build();
//    }
//
//    private Action driveWithIntake(Action driveAction, Intake intake, double power) {
//        return new Action() {
//            private boolean started = false;
//
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                if (!started) {
//                    // Start intake once, when the action begins
//                    intake.setPower(power);
//                    started = true;
//                }
//
//                // Step the drive action
//                boolean driveStillRunning = driveAction.run(packet);
//
//                if (!driveStillRunning) {
//                    // Drive finished -> stop intake and end this wrapper action
//                    intake.setPower(0.0);
//                }
//
//                // This wrapper lives exactly as long as the drive
//                return driveStillRunning;
//            }
//        };
//    }
//
//    private Action withPoseTelemetry(Action inner, MecanumDrive drive, Shooter shooter, boolean allowTrapdoor) {
//        return new Action() {
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                // Continuous shooter update; trapdoor forced closed while moving
//                shooter.update(allowTrapdoor);
//
//                // Telemetry
//                Pose2d pose = drive.localizer.getPose();
//
//                packet.put("X", pose.position.x);
//                packet.put("Y", pose.position.y);
//                packet.put("HeadingDeg", Math.toDegrees(pose.heading.toDouble()));
//                packet.put("Run Time", runtime.toString());
//                packet.put("Target RPM", shooter.getBangBangTargetVelocity());
//                packet.put("Actual RPM", shooter.getActualRPM());
//                packet.put("TrapdoorAllowed", allowTrapdoor);
//
//                telemetry.addData("X", pose.position.x);
//                telemetry.addData("Y", pose.position.y);
//                telemetry.addData("Heading (deg)", Math.toDegrees(pose.heading.toDouble()));
//                telemetry.addData("Run Time", runtime.toString());
//                telemetry.addData("Target RPM", shooter.getBangBangTargetVelocity());
//                telemetry.addData("Actual RPM", shooter.getActualRPM());
//                telemetry.addData("TrapdoorAllowed", allowTrapdoor);
//                telemetry.update();
//
//                return inner.run(packet);
//            }
//        };
//    }
//}
//
//
//
//
//
//
//
////package org.firstinspires.ftc.teamcode;
////
////import androidx.annotation.NonNull;
////
////import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
////import com.acmerobotics.roadrunner.Action;
////import com.acmerobotics.roadrunner.ParallelAction;
////import com.acmerobotics.roadrunner.Pose2d;
////import com.acmerobotics.roadrunner.SequentialAction;
////import com.acmerobotics.roadrunner.Vector2d;
////import com.acmerobotics.roadrunner.ftc.Actions;
////import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
////import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
////import com.qualcomm.robotcore.util.ElapsedTime;
////
////import org.firstinspires.ftc.teamcode.autos.FieldCoordinates;
////import org.firstinspires.ftc.teamcode.autos.TileMoveHelper;
////
////@Autonomous(name = "Test Shockwave Auton Blue Near", group = "Auto")
////public class TestAutonNear extends LinearOpMode {
////
////    private final ElapsedTime runtime = new ElapsedTime();
////
////    @Override
////    public void runOpMode() throws InterruptedException {
////
////        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
////        Shooter nShooter     = new Shooter(hardwareMap, 2800);
////        Shooter fShooter     = new Shooter(hardwareMap, 3500);
////        Intake intake = new Intake(hardwareMap);
////        ShootAndIntakeAction intakeShoot = new ShootAndIntakeAction(nShooter, intake, 2.0, 1.0);
////        ShootAndIntakeAction intakeShootReverse = new ShootAndIntakeAction(nShooter, intake, 0.2, -1.0);
////
////
////        // Show live pose on INIT so you can sanity check Pinpoint signs.
////        while (!isStopRequested() && !opModeIsActive()) {
////            Pose2d p = drive.localizer.getPose();
////            telemetry.addLine("INIT Pose (from Pinpoint / RR):");
////            telemetry.addData("X", p.position.x);
////            telemetry.addData("Y", p.position.y);
////            telemetry.addData("Heading (deg)", Math.toDegrees(p.heading.toDouble()));
////            telemetry.update();
////        }
////        // If this particular starting configuration has flywheel upfield:
////        // "rowsUp" should drive backwards (robot -X), so flip X_SIGN:
////        TileMoveHelper.X_SIGN = -1.0;
////        TileMoveHelper.Y_SIGN = -1.0;
////
////        Pose2d shootingPoseNear, shootingPoseFar;
////
////        waitForStart();
////        if (isStopRequested()) return;
////        runtime.reset();
////        //Pose2d initialPose = FieldCoordinates.BLUE_START_C1;
////        // --------------------------------------------------------------------
////        //        // SIMPLE SHOOT ACTION:
////        //        //
////        //        // Just spin the flywheel using bang-bang for a fixed time.
////        //        // You can tune this duration on the field.
////        //        // --------------------------------------------------------------------
////        double SPIN_AND_SHOOT_TIME = 2.0; // seconds; adjust as needed
////        Action spinAndShoot = nShooter.spinBangBangForTime(SPIN_AND_SHOOT_TIME);
////
////        Action intakeShootComboBall1 = intakeShoot.buildShootIntake();
////        Action intakeShootComboBall2 = intakeShoot.buildShootIntake();
////        Action intakeShootComboBall3 = intakeShoot.buildShootIntake();
////        Action intakeShootComboReverse1 = intakeShootReverse.buildShootIntake();
////        Action intakeShootComboReverse2 = intakeShootReverse.buildShootIntake();
////        Action intakeShootComboReverse3 = intakeShootReverse.buildShootIntake();
////
////        Action spinAndShoot1 = nShooter.spinBangBangForTime(SPIN_AND_SHOOT_TIME);
////
////        Action intakeShootComboBall4 = intakeShoot.buildShootIntake();
////        Action intakeShootComboBall5 = intakeShoot.buildShootIntake();
////        Action intakeShootComboBall6 = intakeShoot.buildShootIntake();
////        Action intakeShootComboReverse4 = intakeShootReverse.buildShootIntake();
////        Action intakeShootComboReverse5 = intakeShootReverse.buildShootIntake();
////        Action intakeShootComboReverse6 = intakeShootReverse.buildShootIntake();
////
////        Action spinAndShoot2 = nShooter.spinBangBangForTime(SPIN_AND_SHOOT_TIME);
////
////        Action intakeShootComboBall7 = intakeShoot.buildShootIntake();
////        Action intakeShootComboBall8 = intakeShoot.buildShootIntake();
////        Action intakeShootComboBall9 = intakeShoot.buildShootIntake();
////        Action intakeShootComboReverse7 = intakeShootReverse.buildShootIntake();
////        Action intakeShootComboReverse8 = intakeShootReverse.buildShootIntake();
////        Action intakeShootComboReverse9 = intakeShootReverse.buildShootIntake();
////
////        Action spinAndShoot3 = nShooter.spinBangBangForTime(SPIN_AND_SHOOT_TIME);
////
////        Action intakeShootComboBall10 = intakeShoot.buildShootIntake();
////        Action intakeShootComboBall11 = intakeShoot.buildShootIntake();
////        Action intakeShootComboBall12 = intakeShoot.buildShootIntake();
////        Action intakeShootComboReverse10 = intakeShootReverse.buildShootIntake();
////        Action intakeShootComboReverse11 = intakeShootReverse.buildShootIntake();
////        Action intakeShootComboReverse12 = intakeShootReverse.buildShootIntake();
////
////
////
////        Action intakeShootCombo2 = intakeShoot.buildShootIntake();
////
////
////        // Shoot from C1 directly
//////        //Start turn and Shoot
//////        Action part1 = drive.actionBuilder(startPose)
//////                .turn(Math.toRadians(35))
//////                .build();
//////
//////        Actions.runBlocking(withPoseTelemetry(part1, drive));
////
////
//// /*       Pose2d p = drive.localizer.getPose();
////        double targetX = p.position.x - 3 * FieldCoordinates.TILE; // if upfield is -X on your robot
////
////        Action toC4 = drive.actionBuilder(p)
////                .lineToX(targetX)
////                .build();
////
////        Actions.runBlocking(withPoseTelemetry(toC4, drive));*/
////
////        //PART 1 Go and Shoot
////        // Starting from wherever the robot is right now
////        Pose2d startPose = drive.localizer.getPose(); // flywheel facing C6 (up)
////
////        // How far is 3 rows in your helper's convention?
////        double deltaFieldForward = 2.75 * TileMoveHelper.TILE;           // 3 tiles
////        double deltaRobotX       = TileMoveHelper.X_SIGN * deltaFieldForward;
////        double targetX           = startPose.position.x + deltaRobotX;
////
////
////
////        // Build ONE trajectory: moveRows(3) then turn right 45°
////        Action moveRows3TurnAndShoot = drive.actionBuilder(startPose)
////                .lineToX(targetX)                 // same math as TileMoveHelper.moveRows(drive, 3)
////                .turn(Math.toRadians(35))        // left = positive angle
////                .build();
////
//////        Action part1ParallelAction = new ParallelAction(
//////                moveRows3TurnAndShoot,
//////                spinAndShoot
//////        );
////        Actions.runBlocking(moveRows3TurnAndShoot);
////
////        shootingPoseNear = drive.localizer.getPose();
////
////        // Part 1: Run it at this shooting pose
////        Actions.runBlocking(withPoseTelemetry(new SequentialAction(
////                intakeShootComboReverse1,
////                intakeShootComboBall1,
////                intakeShootComboReverse2,
////                intakeShootComboBall2,
////                intakeShootComboReverse3,
////                intakeShootComboBall3) , drive, nShooter)
////        );
////
////// ===== after Part 1 =====
////// You already did:
//////   - TileMoveHelper.X_SIGN = -1.0;
//////   - TileMoveHelper.Y_SIGN = -1.0;
//////   - move 3 tiles upfield
//////   - .turn(Math.toRadians(45))  // flywheel -> A6
////// and then:
////        //shootingPoseNear = drive.localizer.getPose(); // C4 shooting pose, heading = flywheel→A6
////
////        telemetry.addLine("Shooting pose recorded (C4-ish)");
////        telemetry.addData("Shoot X", shootingPoseNear.position.x);
////        telemetry.addData("Shoot Y", shootingPoseNear.position.y);
////        telemetry.addData("Shoot H (deg)", Math.toDegrees(shootingPoseNear.heading.toDouble()));
////        telemetry.update();
////
////// ===== PART 2: spin -135° so INTAKE faces A3, then drive forward ====
////
////        // STEP 1: turn the robot
////        // 1) Rotate in place -135° (right turn)
////        //    From your testing: this is what makes the INTAKE point toward A3.
////        Action rotateIntakeToA3 = drive.actionBuilder(shootingPoseNear)
////                .turn(Math.toRadians(-125))   // spin 135° right
////                .build();
////
////        Actions.runBlocking(withPoseTelemetry(rotateIntakeToA3, drive, nShooter));
////
////        //STEP 2: Move robot forward with Intake to A3
////        // 2) Move forward a tile (24") along INTAKE direction
////        Pose2d poseAfterTurn = drive.localizer.getPose();
////        double tile = FieldCoordinates.TILE;
////
////        double deltaRows = 1.5 * tile;
////        double colsLeft = 1.20 * tile;
////
////        // Heading in Pose2d is FLYWHEEL direction; intake is opposite.
////        double flywheelHeading = poseAfterTurn.heading.toDouble();
////        double intakeHeading   = flywheelHeading + Math.PI;
////
////        // Compute world target in front of intake
////        double forwardX = poseAfterTurn.position.x + TileMoveHelper.X_SIGN * deltaRows * Math.cos(intakeHeading);
////        double forwardY = poseAfterTurn.position.y + TileMoveHelper.Y_SIGN * colsLeft * Math.sin(intakeHeading);
////
////        // Drive to that point (still field-centric)
////        Action strafetoA3 = drive.actionBuilder(poseAfterTurn)
////                .strafeTo(new Vector2d(forwardX, forwardY))
////                .build();
////
////        Action part2MoveIntakeParallel = new ParallelAction(
////                strafetoA3,
////                intake.intakeForwardForTime(2.0, 1.0)
////        );
////
////        Actions.runBlocking(withPoseTelemetry(part2MoveIntakeParallel, drive, nShooter));
////
////// ===== spline back to shootingPose (C4 shooting pose) =====
////
////        // STEP 3: Move the robot back to shooting position
////// Pose after Part 2 (near A4, intake forward)
////        Pose2d poseAfterStep2 = drive.localizer.getPose();
////        Action poseBackToShootingPose2 = ShootingPose(shootingPoseNear, drive, poseAfterStep2);
////// Run
////        Action part2MoveShootPosAndShooterStart = new ParallelAction(
////                poseBackToShootingPose2,
////                spinAndShoot1
////        );
////        Actions.runBlocking(withPoseTelemetry(
////                new SequentialAction(
////                        part2MoveShootPosAndShooterStart,
////                        intakeShootComboReverse4,
////                        intakeShootComboBall4,
////                        intakeShootComboReverse5,
////                        intakeShootComboBall5,
////                        intakeShootComboReverse6,
////                        intakeShootComboBall6
////                ),drive, nShooter)
////        );
////
////// End of PART 2
////
////// ===== PART 3: spline from shootingPose (C4) to A2 with INTAKE forward =====
////
////        // --- STEP 1: From C4 shooting pose, turn -45° in place ---
////
////        // This is your requested "turn robot -45" from the shootingPose.
////        Action turnMinus45FromShoot = drive.actionBuilder(shootingPoseNear)
////                .turn(Math.toRadians(-45))   // right turn 45°
////                .build();
////        Actions.runBlocking(withPoseTelemetry(turnMinus45FromShoot, drive, nShooter));
////
////        //STEP 2: Go to A2 with intake running
////
////        // Read pose at C3
////        Pose2d poseAtC3 = drive.localizer.getPose();
////
////        // From C3 (col C, row3) to A2 (col A, row1):
////        //  - 1 row DOWN (row3 -> row2)
////        //  - 2 columns LEFT (C -> A)
////
////        // FIELD semantics:
////        double deltaRowsToA2   = -1 * tile;  //  row down
////        double deltaLeftToA2   =  1.2 * tile;  //  columns left
////
////        // Map to your world frame using the same sign flips:
////        double deltaX_A2 = TileMoveHelper.X_SIGN * deltaRowsToA2; // up/down component
////        double deltaY_A2 = TileMoveHelper.Y_SIGN * deltaLeftToA2; // left/right component
////
////        double targetX_A2 = poseAtC3.position.x + deltaX_A2;
////        double targetY_A2 = poseAtC3.position.y + deltaY_A2;
////
////        // Direction of travel from C3 to A1 in world frame
////        double travelTangent_A2 = Math.atan2(
////                targetY_A2 - poseAtC3.position.y,
////                targetX_A2 - poseAtC3.position.x
////        );
////
////
////        // We still travel along travelTangent, but we want the intake
////        // rotated 45° further to the RIGHT at the end.
////        double headingOffset_A2 = Math.toRadians(-38);  // right turn degrees°
////        double finalHeading_A2  = travelTangent_A2 + headingOffset_A2;
////
////        // Build the A2 pose with INTAKE heading set along travel direction
////        Pose2d a2IntakePose = new Pose2d(
////                targetX_A2,
////                targetY_A2,
////                finalHeading_A2
////        );
////
////        // Spline from C3 pose to A2 intake pose
////        Action splineToA2 = drive.actionBuilder(poseAtC3)
////                .splineToLinearHeading(
////                        a2IntakePose,
////                        travelTangent_A2   // approach tangent along direction of travel
////                )
////                .build();
////
////        Action part3MoveIntakeParallel = new ParallelAction(
////                splineToA2,
////                intake.intakeForwardForTime(4.0, 1.0)
////        );
////        Actions.runBlocking(withPoseTelemetry(part3MoveIntakeParallel, drive, nShooter));
////
////        // STEP 3: Go to Shooting Position and Shoot
////        Pose2d AfterA2 = drive.localizer.getPose();
////        Action poseBackToShootingPoseA2 = ShootingPose(shootingPoseNear,drive,AfterA2);
//////        // Run Part 3
//////        Actions.runBlocking(poseBackToShootingPoseA2);
////
////        Action part3MoveShootPosAndShooterStart = new ParallelAction(
////                poseBackToShootingPoseA2,
////                spinAndShoot2
////        );
////        Actions.runBlocking(withPoseTelemetry(
////                new SequentialAction(
////                        part3MoveShootPosAndShooterStart,
////                        intakeShootComboReverse7,
////                        intakeShootComboBall7,
////                        intakeShootComboReverse8,
////                        intakeShootComboBall8,
////                        intakeShootComboReverse9,
////                        intakeShootComboBall9
////                ), drive, nShooter)
////        );
////
//////END OF PART 3
////
////
////// PART - Pickup from A1 tile
////// ===== PART 4: from C4 shootingPose -> C3 (intake toward C1) -> spline to A1 =====
////
////        // --- STEP 1: From C4 shooting pose, turn -45° in place ---
////        // This is your requested "turn robot -45" from the shootingPose.
////        Action turnMinus45FromShoot2 = drive.actionBuilder(drive.localizer.getPose())
////                .turn(Math.toRadians(-45))   // right turn 45°
////                .build();
////
////        Actions.runBlocking(withPoseTelemetry(turnMinus45FromShoot2, drive, nShooter));
////
////
////        // --- STEP 2: Move one row DOWN (C4 -> C3) with intake facing C1 ---
////
////        // After the turn, read the updated pose.
////        poseAfterTurn = drive.localizer.getPose();
////
////        // In FIELD semantics, "rowsUp" positive is upfield; going from row4 -> row3 is 1 row DOWN:
////        // so rowsUp = -1 => deltaFieldForward = -1 * TILE.
////        double rowsDown = -1.0;
////        double deltaFieldForwardDown = rowsDown * tile;
////
////    // Map this to your world X using X_SIGN (-1.0)
////        double deltaRobotXDown = TileMoveHelper.X_SIGN * deltaFieldForwardDown;
////
////    // Target X for C3 row
////        double targetX_C3 = poseAfterTurn.position.x + deltaRobotXDown;
////
////    // Drive straight along X to that row (still field-centric)
////        Action moveToC3 = drive.actionBuilder(poseAfterTurn)
////                .lineToX(targetX_C3)
////                .build();
////
////        Actions.runBlocking(withPoseTelemetry(moveToC3, drive, nShooter));
////
////        // At this point, you should be roughly at C3, with the same heading
////        // you had after the -45° turn (intake generally facing toward C1).
////
////        // --- STEP 3: From C3, spline to A2 with intake forward ---
////
////        // Read pose at C2
////        Pose2d poseAtC2 = drive.localizer.getPose();
////
////// From C3 (col C, row3) to A1 (col A, row1):
//////  - 1 row DOWN (row3 -> row1)
//////  - 2 columns LEFT (C -> A)
////
////// FIELD semantics:
////        double deltaRowsToA1   = -0.8 * tile;  //  row down
////        double deltaLeftToA1   =  1.2 * tile;  //  columns left
////
////// Map to your world frame using the same sign flips:
////        double deltaX_A1 = TileMoveHelper.X_SIGN * deltaRowsToA1; // up/down component
////        double deltaY_A1 = TileMoveHelper.Y_SIGN * deltaLeftToA1; // left/right component
////
////        double targetX_A1 = poseAtC2.position.x + deltaX_A1;
////        double targetY_A1 = poseAtC2.position.y + deltaY_A1;
////
////// Direction of travel from C3 to A1 in world frame
////        double travelTangent_A1 = Math.atan2(
////                targetY_A1 - poseAtC2.position.y,
////                targetX_A1 - poseAtC2.position.x
////        );
////
//////// For pickup moves we treat heading as INTAKE direction.
//////// So at A1 we want heading == direction of travel.
//////        double finalHeading = travelTangent;
////
////        // We still travel along travelTangent, but we want the intake
////// rotated 45° further to the RIGHT at the end.
////        double headingOffset_A1 = Math.toRadians(-35);  // right turn 45°
////        double finalHeading_A1  = travelTangent_A1 + headingOffset_A1;
////
////// Build the A2 pose with INTAKE heading set along travel direction
////        Pose2d a1IntakePose = new Pose2d(
////                targetX_A1,
////                targetY_A1,
////                finalHeading_A1
////        );
////
////        // Spline from C3 pose to A2 intake pose
////        Action splineToA1 = drive.actionBuilder(poseAtC3)
////                .splineToLinearHeading(
////                        a1IntakePose,
////                        travelTangent_A1   // approach tangent along direction of travel
////                )
////                .build();
////
//////        Actions.runBlocking(splineToA1);
////
////        Action part4MoveIntakeParallel = new ParallelAction(
////                splineToA1,
////                intake.intakeForwardForTime(5.0, 1.0)
////        );
////        Actions.runBlocking(withPoseTelemetry(part4MoveIntakeParallel, drive, nShooter));
////
////        //STEP 3: Go back to shooting position and shoot
////        Pose2d poseAfterA1 = drive.localizer.getPose();
////        Action poseBackToShootingPoseA1 = ShootingPose(shootingPoseNear, drive, poseAfterA1);
////
////        // Run Part 3
//////        Actions.runBlocking(poseBackToShootingPoseA1);
////
////        Action part4MoveShootPosAndShooterStart = new ParallelAction(
////                poseBackToShootingPoseA1,
////                spinAndShoot3
////        );
////        Actions.runBlocking(withPoseTelemetry(
////                new SequentialAction(
////                        part4MoveShootPosAndShooterStart,
////                        intakeShootComboReverse10,
////                        intakeShootComboBall10,
////                        intakeShootComboReverse11,
////                        intakeShootComboBall11,
////                        intakeShootComboReverse12,
////                        intakeShootComboBall12
////                ), drive, nShooter)
////        );
////
////
////    }
////
////    // Generic helper: from any currentPose, build an Action that splines back to shootingPose.
////    private Action ShootingPose(Pose2d shootingPose, MecanumDrive drive, Pose2d currentPose) {
////        if (shootingPose == null) {
////            throw new IllegalStateException("shootingPose is not set before calling buildSplineBackToShootingPose()");
////        }
////
////        // Direction of travel from currentPose back to shootingPose
////        double travelTangent = Math.atan2(
////                shootingPose.position.y - currentPose.position.y,
////                shootingPose.position.x - currentPose.position.x
////        );
////
////        // Build spline:
////        //  - start at currentPose
////        //  - end exactly at shootingPose (x, y, heading)
////        //  - approach along travelTangent
////        return drive.actionBuilder(currentPose)
////                .splineToLinearHeading(
////                        shootingPose,
////                        travelTangent
////                )
////                .build();
////    }
////
////    private double normalizeRadians(double angle) {
////        while (angle > Math.PI)  angle -= 2.0 * Math.PI;
////        while (angle < -Math.PI) angle += 2.0 * Math.PI;
////        return angle;
////    }
////
////    public static class ShootAndIntakeAction {
////
////        private final Shooter shooter;
////        private final Intake intake;
////        private final double durationSeconds;
////        private final double intakeDirection;
////
////        public ShootAndIntakeAction(Shooter shooter, Intake intake, double durationSeconds, double intakeDirection) {
////            this.shooter = shooter;
////            this.intake = intake;
////            this.durationSeconds = durationSeconds;
////            this.intakeDirection = intakeDirection;
////        }
////
////        public Action buildShootIntake() {
////            return new ParallelAction(
////                    shooter.spinBangBangForTime(durationSeconds),
////                    intake.intakeForwardForTime(durationSeconds, intakeDirection)
////            );
////        }
////    }
////
////    private Action withPoseTelemetry(Action inner, MecanumDrive drive, Shooter shooter) {
////        return new Action() {
////            @Override
////            public boolean run(@NonNull TelemetryPacket packet) {
////                // 1) Do telemetry
////                Pose2d pose = drive.localizer.getPose();
////
////                packet.put("X", pose.position.x);
////                packet.put("Y", pose.position.y);
////                packet.put("HeadingDeg", Math.toDegrees(pose.heading.toDouble()));
////                packet.put("Run Time",  runtime.toString());
////                packet.put("Target RPM", shooter.getBangBangTargetVelocity());
////                packet.put("Actual RPM", shooter.getActualRPM());
////
////                telemetry.addData("X", pose.position.x);
////                telemetry.addData("Y", pose.position.y);
////                telemetry.addData("Heading (deg)", Math.toDegrees(pose.heading.toDouble()));
////                telemetry.addData("Run Time", runtime.toString());
////                telemetry.addData("Target RPM", shooter.getBangBangTargetVelocity());
////                telemetry.addData("Actual RPM", shooter.getActualRPM());
////                telemetry.update();
////
////                // 2) Delegate to the wrapped action
////                return inner.run(packet);
////            }
////        };
////    }
////}
