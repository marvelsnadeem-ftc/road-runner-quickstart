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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.autos.FieldCoordinates;
import org.firstinspires.ftc.teamcode.autos.TileMoveHelper;
import org.firstinspires.ftc.teamcode.utils.HelperUtils;

@Autonomous(name = "Shockwave Auton Near Goal Blue", group = "Auto")
public class SWAutonBlueNearGoal extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();

    // Same timing constants as Red Near
    private static final double INTAKE_FORWARD_SEC  = 1.0;
    private static final double INTAKE_REVERSE_SEC  = 0.20;

    // 2) FINAL LOCK (recommended): align tx + hold distance (TeleOp-style)
    private static double TARGET_DISTANCE_CM = 150.0;  // NEAR distance from your TeleOp


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
        //limelight.pipelineSwitch(pipelineSelect); //Blue
        limelight.start();
        // ==========================================
        HelperUtils utils = new HelperUtils(runtime, telemetry, INTAKE_FORWARD_SEC, INTAKE_REVERSE_SEC);
        // -ve for Red(CW), +ve for Blue(CCW) // // set this value only once and only for Auto. utils.setLimelightSearchOmega(0.25); // slow search only during final lock
        utils.setLimelightSearchOmega(0.25);
        utils.setLimelight(limelight);
        utils.setLimelightPipeline(pipelineSelect); //Blue

        // Show live pose during INIT
        while (!isStopRequested() && !opModeIsActive()) {
            Pose2d p = drive.localizer.getPose();
            telemetry.addLine("INIT Pose (from Pinpoint / RR):");
            telemetry.addData("X", p.position.x);
            telemetry.addData("Y", p.position.y);
            telemetry.addData("Heading (deg)", Math.toDegrees(p.heading.toDouble()));
            LLResult r = limelight.getLatestResult();
            telemetry.addData("LL running", true);
            telemetry.addData("LL pipe idx", pipelineSelect);
            telemetry.addData("LL valid", (r != null && r.isValid()));
            telemetry.addData("LL tx", (r != null && r.isValid()) ? r.getTx() : Double.NaN);
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
        //Action burstA5      = utils.buildIntakeBurst(intake, 3);
        Action burstA4      = utils.buildIntakeBurst(intake, 3);
        Action burstA3      = utils.buildIntakeBurst(intake, 3);
        Action burstA2      = utils.buildIntakeBurst(intake, 3);



        // ============================================================
        // PART 1 (BLUE NEW): From B6, drive to C4, turn left toward A6, shoot preloads
        // ============================================================

        // Shooter OFF during Intake + Travel
        nShooter.setEnabled(false);
        // Start pose comes from Pinpoint / RR localizer (robot placed at B6)
        Pose2d startPose = drive.localizer.getPose();

        // Optional (ONLY if your init pose is not already correct at B6):
        // drive.localizer.setPose(new Pose2d(-60, -36, Math.toRadians(180))); // adjust heading if needed

        double tile = FieldCoordinates.TILE;

        // B6 -> C4 is: +2 tiles in X (Row6 -> Row4), +1 tile in Y (B -> C)
        double targetX_C4 = startPose.position.x + 2.0 * tile;
        double targetY_C4 = startPose.position.y + 1.0 * tile;

        // Same aiming math as your working code: A6 relative to C4 is 2 tiles up + 2 tiles left => 45°
//        double aimOffsetRad = Math.atan2(2.0, 2.0); // 45 deg


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
        Action moveToC4WithVision = utils.driveWithLimelightTx(moveToC4, drive);
        // Shooter ON only for the “with vision” return + align + shoot
        nShooter.setEnabled(true);
        utils.setLimelightSearchOmega(0.0);
        Actions.runBlocking(utils.withPoseTelemetry(moveToC4WithVision, drive, nShooter, false));

        // Vision search + lock (blue pipeline selected beforehand)
//        Actions.runBlocking(utils.withPoseTelemetry(
//                utils.alignToTagTx(drive, 1.5),
//                drive, nShooter, false
//        ));

//        // ============================================================
//// PART 1 (BLUE NEW): From A6 (goal tile), drive to C4
//// ============================================================
//
//// Start pose comes from Pinpoint / RR localizer (robot placed at A6)
//        Pose2d startPose = drive.localizer.getPose();
//
//// Optional (ONLY if your init pose is not already correct at A6):
//// A6 tile center in the flywheel-forward map would be (-60, -60), but only setPose if you KNOW you need it.
//// drive.localizer.setPose(new Pose2d(-60, -60, Math.toRadians(180)));
//
//        double tile = FieldCoordinates.TILE;
//
//        double targetX = startPose.position.x + 2.0 * tile;
//        Action moveToC4 = drive.actionBuilder(startPose)
//                .lineToX(targetX)   // straight forward/back along X
//                .build();
//        Actions.runBlocking(utils.withPoseTelemetry(moveToC4, drive, nShooter, false));

        utils.setLimelightSearchOmega(0.25);
        Actions.runBlocking(utils.withPoseTelemetry(
                utils.alignToTagTxAndDistanceCm(drive, TARGET_DISTANCE_CM, 2),
                drive, nShooter, false
        ));


        // Shoot 3 preloads (intake burst only; shooter already running)
        Actions.runBlocking(utils.withPoseTelemetry(
                burstPreload,
                drive, nShooter, true
        ));

        // Record shooting pose AFTER the turn (this is what you return to later)
        shootingPoseNear = drive.localizer.getPose();

        telemetry.addLine("Blue shooting pose recorded (C4-ish pointing A6)");
        telemetry.addData("Shoot X", shootingPoseNear.position.x);
        telemetry.addData("Shoot Y", shootingPoseNear.position.y);
        telemetry.addData("Shoot H (deg)", Math.toDegrees(shootingPoseNear.heading.toDouble()));
        telemetry.update();

//        // ============================================================
//        // PART 2 (BLUE): from shootingPoseNear -> A5|B5 seam midpoint
//        // Turn early by the time we reach B5, then strafe into the seam.
//        // Uses RELATIVE tile deltas from shootingPoseNear.
//        // Convention for THIS Blue start:
//        //   upfield (toward row 6) = X-
//        //   toward A (left)        = Y-
//        // ============================================================
//
//        // Shooter OFF during Intake + Travel
//        nShooter.setEnabled(false);
//
//        // Deltas from C4 -> Row5 and toward A5/B5 seam
//        double rowsUpToRow5      = 1.5;   // C4 -> C5 lane (one row up)
//        double colsTowardB5      = 0.2;   // C -> B (one column toward A)
//        double colsTowardA5B5Mid  = 1.2;   // C -> seam between B and A
//
//        // Convert to inches with your Blue sign convention:
//        // upfield is X-, toward A is Y-
//        double dxRow5   = -rowsUpToRow5     * tile;   // X-
//        double dyB5     = -colsTowardB5     * tile;   // Y-
//        double dyA5B5Seam = -colsTowardA5B5Mid * tile;   // Y-
//
//        // Intermediate point (B5-ish) where we want to be "already turned"
//        double xB5 = shootingPoseNear.position.x + dxRow5;
//        double yB5 = shootingPoseNear.position.y + dyB5;
//
//        // Final pickup target: seam midpoint between A5 and B5 (same row5 X)
//        double xA5Seam = shootingPoseNear.position.x + dxRow5;
//        double yA5Seam = shootingPoseNear.position.y + dyA5B5Seam;
//
//
//
//        // Intake should face toward A (left) = -Y direction => heading -90°
//        // (This matches the “turn early” intent: intake is square before seam)
//        double intakeHeadingTowardA5 = Math.toRadians(-90.0);
//
//        // Tangent into the midpoint (direction of travel start->B5 point)
//        double tangentToB5 = Math.atan2(yB5 - shootingPoseNear.position.y, xB5 - shootingPoseNear.position.x);
//
//        // Build ONE trajectory:
//        //  1) spline to B5-ish point while rotating to intakeHeadingTowardA (turn happens early)
//        //  2) strafe straight into the seam midpoint
//        Pose2d poseAtB5 = new Pose2d(xB5, yB5, intakeHeadingTowardA5);
//
//        Action toA5B5SeamTurnEarly = drive.actionBuilder(shootingPoseNear)
//                .splineToLinearHeading(poseAtB5, tangentToB5)
//                .strafeTo(new Vector2d(xA5Seam, yA5Seam))
//                .build();
//
//        // Intake only while driving (stops exactly when the drive action ends)
//        Action toA5B5SeamWithIntake = utils.driveWithIntake(toA5B5SeamTurnEarly, intake, 1.0);
//        Actions.runBlocking(utils.withPoseTelemetry(toA5B5SeamWithIntake, drive, nShooter, false));
//
//        // Back to shooting pose towards A6, then shoot the 3 pickup balls
//        Pose2d poseAtA5 = drive.localizer.getPose();
//
//        // 0) CLEAR the seam first (move away from A wall / seam so you don't clip A4|B4 balls)
//        // Go back to the B5-ish intermediate point you already used
//        Action clearA5Seam = drive.actionBuilder(poseAtA5)
//                .strafeTo(new Vector2d(xB5, yB5))
//                .build();
//
////        // 0) CLEAR: step away from the B5 ball before heading back
////        // Safe offsets (tune small):
////        //   - push slightly toward C (Y+) so we don't pass through B5
////        //   - and/or push slightly upfield (X-) so we are above the ball line
////        double clearY = 0.35 * tile;  // ~8.4" toward C (since toward C is +Y from B in your convention)
////        double clearX = 0.20 * tile;  // ~4.8" upfield (X-)
////
////        double xClear = xB5 + (-clearX);   // X- is upfield
////        double yClear = yB5 + (+clearY);   // Y+ is toward C (away from A wall / B5 ball)
////
////        // Go to the safe-clear point instead of B5
////        Action clearA5Seam = drive.actionBuilder(poseAtA5)
////                .strafeTo(new Vector2d(xClear, yClear))
////                .build();
//
//
//        Actions.runBlocking(utils.withPoseTelemetry(clearA5Seam, drive, nShooter, false));
//
//        // 1) Return to shootingPoseNear (with vision assist while driving)
//        Pose2d poseAfterB5Clear = drive.localizer.getPose();
//        Action poseBackToShootingPoseFromA5 = utils.GoToPose(poseAfterB5Clear, drive, shootingPoseNear);
//
//        // IMPORTANT: prevent “search spin” while following if tag flickers
//        utils.setLimelightSearchOmega(0.0);
//        Action poseBackToShootingPoseFromA5WithVision = utils.driveWithLimelightTx(poseBackToShootingPoseFromA5, drive);
//        // Shooter ON only for the “with vision” return + align + shoot
//        nShooter.setEnabled(true);
//        Actions.runBlocking(utils.withPoseTelemetry(poseBackToShootingPoseFromA5WithVision, drive, nShooter, false));
//
//
//////        // Back to shooting pose towards A6, then shoot the 3 pickup balls
//////        Pose2d poseAtA5 = drive.localizer.getPose();
//////        Action poseBackToShootingPoseFromA5 = utils.GoToPose(poseAtA5, drive, shootingPoseNear); //GoToPose(fromPose, drive, toPose) Go to from currentpose to a new topose
//////        Action poseBackToShootingPoseFromA5WithVision = utils.driveWithLimelightTx(poseBackToShootingPoseFromA5, drive);
//////        Actions.runBlocking(utils.withPoseTelemetry(poseBackToShootingPoseFromA5WithVision, drive, nShooter, false));
//
//
////        // Vision search + lock (blue pipeline selected beforehand)
////        Actions.runBlocking(utils.withPoseTelemetry(
////                utils.alignToTagTx(drive, 1.5),
////                drive, nShooter, false
////        ));
//
//        // IMPORTANT: prevent “search spin” while following if tag flickers
//        utils.setLimelightSearchOmega(0.25);
//        Actions.runBlocking(utils.withPoseTelemetry(
//                utils.alignToTagTxAndDistanceCm(drive, TARGET_DISTANCE_CM, 2),
//                drive, nShooter, false
//        ));
//
//        // 3) Shoot
//        Actions.runBlocking(utils.withPoseTelemetry(
//                new SequentialAction(burstA5),
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

        // Shooter OFF during Intake + Travel
        nShooter.setEnabled(false);

        // Deltas from C4 and toward A4/B4 seam
        double rowsUpToRow4      = 0.65;   // C4 -> C4 lane (same lane)
        double colsTowardB4      = 0.2;   // C -> B (one column toward A)
        double colsTowardB4A4Mid  = 1.2;   // C -> seam between B and A

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


//        // 0) CLEAR the seam first (move away from A wall / seam so you don't clip A4|B4 balls)
//        // Go back to the B5-ish intermediate point you already used
//        Action clearA4Seam = drive.actionBuilder(poseAtA4)
//                .strafeTo(new Vector2d(xB4, yB4))
//                .build();
//        Actions.runBlocking(utils.withPoseTelemetry(clearA4Seam, drive, nShooter, false));
//
//        // 1) Return to shootingPoseNear (with vision assist while driving)
//        Pose2d poseAfterB4Clear = drive.localizer.getPose();
//        Action poseBackToShootingPoseFromA4 = utils.GoToPose(poseAfterB4Clear, drive, shootingPoseNear);

        Action poseBackToShootingPoseFromA4 = utils.GoToPose(poseAtA4, drive, shootingPoseNear); //GoToPose(fromPose, drive, toPose) Go to from currentpose to a new topose

        utils.setLimelightSearchOmega(0.0);
        Action poseBackToShootingPoseFromA4WithVision = utils.driveWithLimelightTx(poseBackToShootingPoseFromA4, drive);
        // Shooter ON only for the “with vision” return + align + shoot
        nShooter.setEnabled(true);
        Actions.runBlocking(utils.withPoseTelemetry(poseBackToShootingPoseFromA4WithVision, drive, nShooter, false));

        // IMPORTANT: prevent “search spin” while following if tag flickers
        utils.setLimelightSearchOmega(0.25);
        Actions.runBlocking(utils.withPoseTelemetry(
                utils.alignToTagTxAndDistanceCm(drive, TARGET_DISTANCE_CM, 2),
                drive, nShooter, false
        ));

        Actions.runBlocking(utils.withPoseTelemetry(
                new SequentialAction(
                        burstA4
                ),
                drive, nShooter, true
        ));

        // ============================================================
        // PART 4 (BLUE): from shootingPoseNear -> A3|B3 seam midpoint
        // Turn early by the time we reach B3, then strafe into the seam.
        // Uses RELATIVE tile deltas from shootingPoseNear.
        // Convention for THIS Blue start:
        //   downfield (toward row 3) = X+
        //   toward A (left)        = Y-
        // ============================================================

        // Shooter OFF during Intake + Travel
        nShooter.setEnabled(false);

        // Deltas from C3 and toward A3/B3 seam
        double rowsDownToRow3      = 0.20;   // C4 -> C3 lane (one row down)
        double colsTowardB3      = 0.5;   // C -> B (one column toward A)
        double colsTowardB3A3Mid  = 1.2;   // C -> seam between B and A

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
        utils.setLimelightSearchOmega(0.0);
        // Shooter ON only for the “with vision” return + align + shoot
        nShooter.setEnabled(true);
        Action poseBackToShootingPoseFromA3WithVision = utils.driveWithLimelightTx(poseBackToShootingPoseFromA3, drive);
        Actions.runBlocking(utils.withPoseTelemetry(poseBackToShootingPoseFromA3WithVision, drive, nShooter, false));

        utils.setLimelightSearchOmega(0.25);
        Actions.runBlocking(utils.withPoseTelemetry(
                utils.alignToTagTxAndDistanceCm(drive, TARGET_DISTANCE_CM, 2),
                drive, nShooter, false
        ));

        Actions.runBlocking(utils.withPoseTelemetry(
                new SequentialAction(
                        burstA3
                ),
                drive, nShooter, true
        ));

//        // ============================================================
//        // PART 5 (BLUE): from shootingPoseNear -> A2|B2 seam midpoint
//        // Turn early by the time we reach B2, then strafe into the seam.
//        // Uses RELATIVE tile deltas from shootingPoseNear.
//        // Convention for THIS Blue start:
//        //   downfield (toward row 2) = X+
//        //   toward A (left)        = Y-
//        // ============================================================
//
//        // Shooter OFF during Intake + Travel
//        nShooter.setEnabled(false);
//
//        // Deltas from C2 and toward A2/B2 seam
//        double rowsDownToRow2      = 1.3;   // C4 -> C2 lane (one row down)
//        double colsTowardB2      = 0.5;   // C -> B (one column toward A)
//        double colsTowardB2A2Mid  = 1.2;   // C -> seam between B and A
//
//        // Convert to inches with your Blue sign convention:
//        // downfield is X+, toward A is Y-
//        double dxRow2   = rowsDownToRow2     * tile;   // X+
//        double dyB2     = -colsTowardB2     * tile;   // Y-
//        double dyA2B2Seam = -colsTowardB2A2Mid * tile;   // Y-
//
//        // Intermediate point (B2-ish) where we want to be "already turned"
//        double xB2 = shootingPoseNear.position.x + dxRow2;
//        double yB2 = shootingPoseNear.position.y + dyB2;
//
//        // Final pickup target: seam midpoint between A5 and B5 (same row5 X)
//        double x2Seam = shootingPoseNear.position.x + dxRow2;
//        double y2Seam = shootingPoseNear.position.y + dyA2B2Seam;
//
//        // Intake should face toward A (left) = -Y direction => heading -90°
//        // (This matches the “turn early” intent: intake is square before seam)
//        double intakeHeadingTowardA2 = Math.toRadians(-90.0);
//
//        // Tangent into the midpoint (direction of travel start->B5 point)
//        double tangentToB2 = Math.atan2(yB2 - shootingPoseNear.position.y, xB2 - shootingPoseNear.position.x);
//
//        // Build ONE trajectory:
//        //  1) spline to B5-ish point while rotating to intakeHeadingTowardA (turn happens early)
//        //  2) strafe straight into the seam midpoint
//        Pose2d poseAtB2 = new Pose2d(xB2, yB2, intakeHeadingTowardA2);
//
//        Action toA2B2SeamTurnEarly = drive.actionBuilder(shootingPoseNear)
//                .splineToLinearHeading(poseAtB2, tangentToB2)
//                .strafeTo(new Vector2d(x2Seam, y2Seam))
//                .build();
//
//        // Intake only while driving (stops exactly when the drive action ends)
//        Action toA2B2SeamWithIntake = utils.driveWithIntake(toA2B2SeamTurnEarly, intake, 1.0);
//        Actions.runBlocking(utils.withPoseTelemetry(toA2B2SeamWithIntake, drive, nShooter, false));
//
//        // Back to shooting pose towards A6, then shoot the 3 pickup balls
//        Pose2d poseAtA2 = drive.localizer.getPose();
//        Pose2d shootingPoseFar = new Pose2d(110.7995, 25, Math.toRadians(25));
//        Action poseBackToShootingPoseFromA2 = utils.GoToPose(poseAtA2, drive, shootingPoseFar); //GoToPose(fromPose, drive, toPose) Go to from currentpose to a new topose
//        utils.setLimelightSearchOmega(0.0);
//        Action poseBackToShootingPoseFromA2WithVision = utils.driveWithLimelightTx(poseBackToShootingPoseFromA2, drive);
//        // Shooter ON only for the “with vision” return + align + shoot
//        fShooter.setEnabled(true);
//        Actions.runBlocking(utils.withPoseTelemetry(poseBackToShootingPoseFromA2WithVision, drive, nShooter, false));
//        utils.setLimelightSearchOmega(0.25);
//        Actions.runBlocking(utils.withPoseTelemetry(
//                utils.alignToTagTxAndDistanceCm(drive, 295, 2),
//                drive, fShooter, false
//        ));
//
//        Actions.runBlocking(utils.withPoseTelemetry(
//                new SequentialAction(
//                        burstA2
//                ),
//                drive, fShooter, true
//        ));
//
//
//        // Optional: record for telemetry/debug
//        shootingPoseFar = drive.localizer.getPose();

        // Shooter OFF Intake + Travel
        fShooter.setEnabled(false);

        // Shooter OFF Intake + Travel
        nShooter.setEnabled(false);
        //shootingPoseFar = drive.localizer.getPose();

    }

}
