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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.autos.FieldCoordinates;
import org.firstinspires.ftc.teamcode.autos.TileMoveHelper;
import org.firstinspires.ftc.teamcode.utils.HelperUtils;

@Autonomous(name = "Shockwave Auton Far Red", group = "Auto")
public class SWAutonRedFar extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();

    // === TIMING CONSTANTS (same as Blue, tune on field if needed) ===
    private static final double INTAKE_FORWARD_SEC  = 1.25;
    private static final double INTAKE_REVERSE_SEC  = 0.30;

    // 2) FINAL LOCK (recommended): align tx + hold distance (TeleOp-style)
    private static double TARGET_DISTANCE_CM = 150.0;  // NEAR distance from your TeleOp
    private static double TARGET_DISTANCE_CM_FAR = 295;

    @Override
    public void runOpMode() throws InterruptedException {

        // Seed pose; Pinpoint / RR localizer will give us the live pose.
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        // Shooter runs continuously with bang-bang, same as Blue
        Shooter nShooter = new Shooter(hardwareMap, 2800);
        Shooter fShooter = new Shooter(hardwareMap, 3500);
        Intake  intake  = new Intake(hardwareMap);

        // ===== ADDED: Limelight init (minimum) =====
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        int pipelineSelect = 1; // 0 is blue, 1 is for red
        limelight.pipelineSwitch(pipelineSelect); //Red
        limelight.start();
        // ==========================================

        HelperUtils utils = new HelperUtils(runtime, telemetry, INTAKE_FORWARD_SEC, INTAKE_REVERSE_SEC);
        utils.setLimelightSearchOmega(-1.2); // -ve for Red, +ve for Blue // set this value only once.
        utils.setLimelight(limelight);
        utils.setLimelightPipeline(pipelineSelect);

        // Show live pose during INIT for sanity check
        while (!isStopRequested() && !opModeIsActive()) {
            Pose2d p = drive.localizer.getPose();
            telemetry.addLine("INIT Pose (from Pinpoint / RR):");
            telemetry.addData("X", p.position.x);
            telemetry.addData("Y", p.position.y);
            telemetry.addData("Heading (deg)", Math.toDegrees(p.heading.toDouble()));
            telemetry.update();
        }

        // Same sign config as Blue (field-centric upfield = -X, left = -Y in your odom)
        TileMoveHelper.X_SIGN = -1.0;
        TileMoveHelper.Y_SIGN = -1.0;
        double tile = FieldCoordinates.TILE;
        Pose2d shootingPoseNear;

        waitForStart();
        if (isStopRequested()) return;

        runtime.reset();

        // Build bursts as needed (fresh action each time)
        Action burstPreload = utils.buildIntakeBurst(intake, 3);
        Action burstF5      = utils.buildIntakeBurst(intake, 3);
        Action burstF4      = utils.buildIntakeBurst(intake, 3);
        Action burstF3      = utils.buildIntakeBurst(intake, 3);
        Action burstF2      = utils.buildIntakeBurst(intake, 3);


        Pose2d startPose = drive.localizer.getPose();

        Pose2d shootingPoseFar = new Pose2d(
                115.0493,          // X
                -25.0,            // Y
                Math.toRadians(-25)  // Heading: intake downfield, flywheel upfield toward D6
        );

// ============================
// PART 1: Vision align + hold 298 cm, then shoot preloads
// ============================
        fShooter.setEnabled(true);

// For RED goal alignment, make sure search omega is correct sign for your robot
        utils.setLimelightSearchOmega(-0.25);

// This should rotate toward the A6 tag and regulate distance to 298 cm
        Actions.runBlocking(utils.withPoseTelemetry(
                utils.alignToTagTxAndDistanceCm(drive, TARGET_DISTANCE_CM_FAR, 2.0),
                drive, fShooter, false
        ));

// Shoot 3 preloads
        Actions.runBlocking(utils.withPoseTelemetry(
                burstPreload,
                drive, fShooter, true
        ));

// Record the “far” shooting pose after vision alignment (authoritative for the rest)
        shootingPoseFar = drive.localizer.getPose();



        //-----------------------------------------------------
        /// PART 2
        //-------------------------------------------------------

        fShooter.setEnabled(false);

        double rowsUpF2      = 0.35;   // upfield => -X
        double colsTowardF2  = 2.00;   // toward F => +Y
        double colsTowardE2  = 1.00;   // D->E

        double dxF2 = -rowsUpF2 * tile;
        double dyF2 =  colsTowardF2 * tile;
        double dyE2 =  colsTowardE2 * tile;

        double xStart = shootingPoseFar.position.x;
        double yStart = shootingPoseFar.position.y;

// E2 and F2 target points (relative)
        double xE2 = xStart + dxF2;
        double yE2 = yStart + dyE2;

        double xF2 = xStart + dxF2;
        double yF2 = yStart + dyF2;

        double intakeHeading = Math.toRadians(90.0);

// Early-turn point: partway to E2 (turn should be done by here)
        double turnFrac = 0.55; // 0.4–0.7 is typical
        double xTurn = xStart + turnFrac * (xE2 - xStart);
        double yTurn = yStart + turnFrac * (yE2 - yStart);

        double tangentToTurn = Math.atan2(yTurn - yStart, xTurn - xStart);
        double tangentToE2   = Math.atan2(yE2 - yTurn, xE2 - xTurn);

        Pose2d poseAtTurn = new Pose2d(xTurn, yTurn, intakeHeading);
        Pose2d poseAtE2   = new Pose2d(xE2,   yE2,   intakeHeading);

        Action toF2_TurnDoneEarly = drive.actionBuilder(shootingPoseFar)
                // Segment 1: spline while rotating; heading reaches intakeHeading by poseAtTurn
                .splineToLinearHeading(poseAtTurn, tangentToTurn)
                // Segment 2: continue to E2 holding intakeHeading
                .splineToLinearHeading(poseAtE2, tangentToE2)
                // Segment 3: strafe into F2 still holding intakeHeading
                .strafeTo(new Vector2d(xF2, yF2))
                .build();

        Action toF2WithIntake = utils.driveWithIntake(toF2_TurnDoneEarly, intake, 1.0);
        Actions.runBlocking(utils.withPoseTelemetry(toF2WithIntake, drive, fShooter, false));

        // Return to ShootingPose from Far
        Pose2d poseAtF2 = drive.localizer.getPose();
//        Action poseBackToShootingPoseFromF4 = GoToPose(poseAtF4, drive, shootingPoseNear); //GoToPose(fromPose, drive, toPose) Go to from currentpose to a new topose
//        Actions.runBlocking(withPoseTelemetry(poseBackToShootingPoseFromF4, drive, nShooter, false));

        Action poseBackToShootingPoseFromF2 = utils.GoToPose(poseAtF2, drive, shootingPoseFar);
        Action poseBackToShootingPoseFromF2WithVision = utils.driveWithLimelightTx(poseBackToShootingPoseFromF2, drive);
        utils.setLimelightSearchOmega(0.0);
        // Shooter ON only for the “with vision” return + align + shoot
        fShooter.setEnabled(true);
        Actions.runBlocking(utils.withPoseTelemetry(poseBackToShootingPoseFromF2WithVision, drive, fShooter, false));

        // IMPORTANT: prevent “search spin” while following if tag flickers
        utils.setLimelightSearchOmega(-0.25);
        Actions.runBlocking(utils.withPoseTelemetry(
                utils.alignToTagTxAndDistanceCm(drive, TARGET_DISTANCE_CM_FAR, 2),
                drive, fShooter, false
        ));


        Actions.runBlocking(utils.withPoseTelemetry(
                new SequentialAction(
                        burstF2
                ),
                drive, fShooter, true
        ));

        fShooter.setEnabled(false);


        //-----------------------------------------------------
        /// PART 3  (go to F3, turn by mid of E3)  -- UNIQUE variable names
        //-------------------------------------------------------

        fShooter.setEnabled(false);

        // ---- UNIQUE Part 3 names (so they don't collide with Part 2) ----
        double rowsUpF3_P3      = 1.35;   // upfield => -X
        double colsTowardF3_P3  = 2.00;   // toward F => +Y
        double colsTowardE3_P3  = 1.00;   // D->E

        double dxF3_P3 = -rowsUpF3_P3 * tile;
        double dyF3_P3 =  colsTowardF3_P3 * tile;
        double dyE3_P3 =  colsTowardE3_P3 * tile;

        double xStart_P3 = shootingPoseFar.position.x;
        double yStart_P3 = shootingPoseFar.position.y;

        // E3 and F3 target points (relative)
        double xE3_P3 = xStart_P3 + dxF3_P3;
        double yE3_P3 = yStart_P3 + dyE3_P3;

        double xF3_P3 = xStart_P3 + dxF3_P3;
        double yF3_P3 = yStart_P3 + dyF3_P3;

        double intakeHeading_P3 = Math.toRadians(90.0);

        // Early-turn point: partway to E3 (turn should be done by here)
        double turnFrac_P3 = 0.55; // 0.4–0.7 is typical
        double xTurn_P3 = xStart_P3 + turnFrac_P3 * (xE3_P3 - xStart_P3);
        double yTurn_P3 = yStart_P3 + turnFrac_P3 * (yE3_P3 - yStart_P3);

        double tangentToTurn_P3 = Math.atan2(yTurn_P3 - yStart_P3, xTurn_P3 - xStart_P3);
        double tangentToE3_P3   = Math.atan2(yE3_P3 - yTurn_P3, xE3_P3 - xTurn_P3);

        Pose2d poseAtTurn_P3 = new Pose2d(xTurn_P3, yTurn_P3, intakeHeading_P3);
        Pose2d poseAtE3_P3   = new Pose2d(xE3_P3,   yE3_P3,   intakeHeading_P3);

        Action toF3_TurnDoneEarly_P3 = drive.actionBuilder(shootingPoseFar)
                // Segment 1: spline while rotating; heading reaches intakeHeading by poseAtTurn
                .splineToLinearHeading(poseAtTurn_P3, tangentToTurn_P3)
                // Segment 2: continue to E3 holding intakeHeading
                .splineToLinearHeading(poseAtE3_P3, tangentToE3_P3)
                // Segment 3: strafe into F3 still holding intakeHeading
                .strafeTo(new Vector2d(xF3_P3, yF3_P3))
                .build();

        Action toF3WithIntake_P3 = utils.driveWithIntake(toF3_TurnDoneEarly_P3, intake, 1.0);
        Actions.runBlocking(utils.withPoseTelemetry(toF3WithIntake_P3, drive, fShooter, false));

        // Return to ShootingPose from Far
        Pose2d poseAtF3_P3 = drive.localizer.getPose();

        Action poseBackToShootingPoseFromF3_P3 = utils.GoToPose(poseAtF3_P3, drive, shootingPoseFar);
        Action poseBackToShootingPoseFromF3WithVision_P3 = utils.driveWithLimelightTx(poseBackToShootingPoseFromF3_P3, drive);
        utils.setLimelightSearchOmega(0.0);

        // Shooter ON only for the “with vision” return + align + shoot
        fShooter.setEnabled(true);
        Actions.runBlocking(utils.withPoseTelemetry(poseBackToShootingPoseFromF3WithVision_P3, drive, fShooter, false));

        // IMPORTANT: prevent “search spin” while following if tag flickers
        utils.setLimelightSearchOmega(-0.25);
        Actions.runBlocking(utils.withPoseTelemetry(
                utils.alignToTagTxAndDistanceCm(drive, TARGET_DISTANCE_CM_FAR, 2),
                drive, fShooter, false
        ));

        Actions.runBlocking(utils.withPoseTelemetry(
                new SequentialAction(
                        burstF3
                ),
                drive, fShooter, true
        ));

        fShooter.setEnabled(false);






//        // ============================================================
//        // PART 1: From D1-ish, drive upfield and rotate shooter toward F6, shoot preloads
//        // ============================================================
//
//        Pose2d startPose = drive.localizer.getPose(); // robot placed at D1, shooter facing D6 (upfield)
//
//        // Shooter OFF during intake + travel
//        nShooter.setEnabled(false);
//
//        double deltaFieldForward = 3.15 * TileMoveHelper.TILE;     // same distance as Blue
//        double deltaRobotX       = TileMoveHelper.X_SIGN * deltaFieldForward;
//        double targetX           = startPose.position.x + deltaRobotX;
//
//        // .turn(-40) so we rotate toward F6 (right goal)
////        Action moveToD4TurnAndShootPose = drive.actionBuilder(startPose)
////                .lineToX(targetX)
////                .turn(Math.toRadians(-40))    // turn right toward F6 instead of left toward A6
////                .build();
////
////        Action moveToD4TurnAndShootPoseWithVision = utils.driveWithLimelightTx(moveToD4TurnAndShootPose, drive);
////        Actions.runBlocking(
////                utils.withPoseTelemetry(moveToD4TurnAndShootPoseWithVision, drive, nShooter, false)
////        );
//
//        Action moveToD4 = drive.actionBuilder(startPose)
//                .lineToX(targetX)
//                .build();
//
//        Action moveToD4WithVision = utils.driveWithLimelightTx(moveToD4, drive);
//        // Shooter ON only for the “with vision” return + align + shoot
//        nShooter.setEnabled(true);
//        utils.setLimelightSearchOmega(0.0);
//        Actions.runBlocking(utils.withPoseTelemetry(moveToD4WithVision, drive, nShooter, false));
//
////        // Vision search + lock (red pipeline selected beforehand)
////        Actions.runBlocking(utils.withPoseTelemetry(
////                utils.alignToTagTx(drive, 1.5),
////                drive, nShooter, false
////        ));
//
//        utils.setLimelightSearchOmega(-0.25);
//        Actions.runBlocking(utils.withPoseTelemetry(
//                utils.alignToTagTxAndDistanceCm(drive, TARGET_DISTANCE_CM, 2),
//                drive, fShooter, false
//        ));
//
//        // Shoot 3 preloads with intake-only burst (shooter already running)
//        Actions.runBlocking(utils.withPoseTelemetry(
//                burstPreload,
//                drive, nShooter, true
//        ));
//
//        shootingPoseNear = drive.localizer.getPose();
//
//        telemetry.addLine("Red shooting pose recorded (D4-ish pointing F6)");
//        telemetry.addData("Shoot X", shootingPoseNear.position.x);
//        telemetry.addData("Shoot Y", shootingPoseNear.position.y);
//        telemetry.addData("Shoot H (deg)", Math.toDegrees(shootingPoseNear.heading.toDouble()));
//        telemetry.update();
//
//        // ============================================================
//        // PART 2: Rotate robot towards D5 intake facing F5 from ShootingPose, drive and intake in artifacts from F5, return and shoot
//        //
//        // ============================================================
//
//        // Shooter OFF during intake + travel
//        nShooter.setEnabled(false);
//
//        Action rotateIntakeToD5 = drive.actionBuilder(shootingPoseNear)
//                .turn(Math.toRadians(105)) //115
//                .build();
//
//        Actions.runBlocking(utils.withPoseTelemetry(rotateIntakeToD5, drive, nShooter, false));
//
//
//        double tilesForwardToF5 = -1.75;  // D5 -> F5 is 2 tiles
//
//        Pose2d poseAtD5 = drive.localizer.getPose();   // near D5
//
//        double targetY_F5 = poseAtD5.position.y - tilesForwardToF5 * tile;  // -12 - 48 = -60
//
//        Action toF5Center = drive.actionBuilder(poseAtD5)
//                .strafeTo(new Vector2d(poseAtD5.position.x, targetY_F5))
//                .build();
//
//        // With intake running while driving
//        Action driveToF5WithIntake = utils.driveWithIntake(toF5Center, intake, 1.0);
//        Actions.runBlocking(utils.withPoseTelemetry(driveToF5WithIntake, drive, nShooter, false));
//
//
//        // Back to shooting pose near F6, then shoot the 3 pickup balls
//        Pose2d poseAtF5 = drive.localizer.getPose();
//
//        // STEP 1: Backtrack from F5 -> D5 lane (avoid crossing E4|F4 seam)
//        Action backToD5Lane = drive.actionBuilder(poseAtF5)
//                .strafeTo(new Vector2d(poseAtF5.position.x, poseAtD5.position.y))
//                .build();
//
//        Actions.runBlocking(utils.withPoseTelemetry(backToD5Lane, drive, nShooter, false));
//
//        Pose2d poseBackAtD5 = drive.localizer.getPose();
//
//        // STEP 2: From D5 -> shootingPoseNear (now you're in a clear lane)
//        Action poseBackToShootingPoseFromD5 = utils.GoToPose(poseBackAtD5, drive, shootingPoseNear);
//
//        // Vision-assist on the way back (optional but recommended)
//        Action poseBackToShootingPoseFromD5WithVision = utils.driveWithLimelightTx(poseBackToShootingPoseFromD5, drive);
//        // Shooter ON only for the “with vision” return + align + shoot
//        nShooter.setEnabled(true);
//        utils.setLimelightSearchOmega(0.0);
//        Actions.runBlocking(utils.withPoseTelemetry(poseBackToShootingPoseFromD5WithVision, drive, nShooter, false));
//
//        utils.setLimelightSearchOmega(-0.25);
//        Actions.runBlocking(utils.withPoseTelemetry(
//                utils.alignToTagTxAndDistanceCm(drive, TARGET_DISTANCE_CM, 2),
//                drive, fShooter, false
//        ));
//
//        Actions.runBlocking(utils.withPoseTelemetry(
//                new SequentialAction(
//                        burstF5
//                ),
//                drive, nShooter, true
//        ));
//


//        // ============================================================
//        // PART 3 (RED): from D5 intake-facing-F5 -> F4 (one step)
//        // Authoritative convention (flywheel-forward map):
//        //   upfield (toward row 6) = -X
//        //   toward F (column F)    = +Y
//        // ============================================================
//
//        // Shooter OFF during intake + travel
//        nShooter.setEnabled(false);
//
//        // Your Part3 target (example: D4 -> F3)
//        double rowsDownF4    = 1.2;   // +X
//        double colsTowardF4 = 1.75;   // +Y
//
//        // We want the robot to already be turned by E4 (midpoint):
//        double colsTowardE4Mid = 1.0;  // D -> E
//
//        // Convert to inches
//        double dxF4 = rowsDownF4 * tile;
//        double dyF4 = colsTowardF4 * tile;
//        double dyMidE4 = colsTowardE4Mid * tile;
//
//        // Target positions (LOCAL / relative)
//        double xE4 = shootingPoseNear.position.x + dxF4;
//        double yE4 = shootingPoseNear.position.y + dyMidE4;
//
//        double xF4 = shootingPoseNear.position.x + dxF4;
//        double yF4 = shootingPoseNear.position.y + dyF4;
//
//        // Desired intake heading toward F (+Y)
//        double intakeHeadingF4 = Math.toRadians(90.0);
//
//        // Tangent into the midpoint (direction of travel start->E4)
//        double tangentToE4 = Math.atan2(yE4 - shootingPoseNear.position.y, xE4 - shootingPoseNear.position.x);
//
//        // Build ONE trajectory:
//        //  1) spline to "E4" while rotating to intakeHeading (turn happens early)
//        //  2) strafe straight to F4 with heading already correct
//        Pose2d poseAtE4 = new Pose2d(xE4, yE4, intakeHeadingF4);
//
//        Action toF4TurnEarly = drive.actionBuilder(shootingPoseNear)
//                .splineToLinearHeading(poseAtE4, tangentToE4)
//                .strafeTo(new Vector2d(xF4, yF4))
//                .build();
//
//        // Intake only while driving
//        Action toF4WithIntake = utils.driveWithIntake(toF4TurnEarly, intake, 1.0);
//        Actions.runBlocking(utils.withPoseTelemetry(toF4WithIntake, drive, nShooter, false));
//
//        // Back to shooting pose near F6, then shoot the 3 pickup balls
//        Pose2d poseAtF4 = drive.localizer.getPose();
//
//        Action poseBackToShootingPoseFromF4 = utils.GoToPose(poseAtF4, drive, shootingPoseNear); //utils.GoToPose(fromPose, drive, toPose) Go to from currentpose to a new topose
//        Action poseBackToShootingPoseFromF4WithVision = utils.driveWithLimelightTx(poseBackToShootingPoseFromF4, drive);
//
//        // Shooter ON only for the “with vision” return + align + shoot
//        nShooter.setEnabled(true);
//        utils.setLimelightSearchOmega(0.0);
//        Actions.runBlocking(utils.withPoseTelemetry(poseBackToShootingPoseFromF4WithVision, drive, nShooter, false));
//
//        utils.setLimelightSearchOmega(-0.25);
//        Actions.runBlocking(utils.withPoseTelemetry(
//                utils.alignToTagTxAndDistanceCm(drive, TARGET_DISTANCE_CM, 2),
//                drive, fShooter, false
//        ));
//
//        Actions.runBlocking(utils.withPoseTelemetry(
//                new SequentialAction(
//                        burstF4
//                ),
//                drive, nShooter, true
//        ));
//
//
//        // ============================================================
//        // PART 4 (RED): ShootingPose (current pose) -> F3 (relative tiles)
//        // One trajectory: rotate to intake-facing-F while moving
//        // No separate .turn() step
//        // ============================================================
//
//        // Shooter OFF during intake + travel
//        nShooter.setEnabled(false);
//
//        // Your Part4 target (example: D4 -> F3)
//        double rowsDownF3    = 2.2;   // +X
//        double colsTowardF3 = 1.75;   // +Y
//
//        // We want the robot to already be turned by E3 (midpoint):
//        double colsTowardE3Mid = 1.0;  // D -> E
//
//        // Convert to inches
//        double dxF3 = rowsDownF3 * tile;
//        double dyF3 = colsTowardF3 * tile;
//        double dyMidE3 = colsTowardE3Mid * tile;
//
//        // Target positions (LOCAL / relative)
//        double xE3 = shootingPoseNear.position.x + dxF3;
//        double yE3 = shootingPoseNear.position.y + dyMidE3;
//
//        double xF3 = shootingPoseNear.position.x + dxF3;
//        double yF3 = shootingPoseNear.position.y + dyF3;
//
//        // Desired intake heading toward F (+Y)
//        double intakeHeadingF3 = Math.toRadians(90.0);
//
//        // Tangent into the midpoint (direction of travel start->E3)
//        double tangentToE3 = Math.atan2(yE3 - shootingPoseNear.position.y, xE3 - shootingPoseNear.position.x);
//
//        // Build ONE trajectory:
//        //  1) spline to "E3" while rotating to intakeHeading (turn happens early)
//        //  2) strafe straight to F3 with heading already correct
//        Pose2d poseAtE3 = new Pose2d(xE3, yE3, intakeHeadingF3);
//
//        Action toF3TurnEarly = drive.actionBuilder(shootingPoseNear)
//                .splineToLinearHeading(poseAtE3, tangentToE3)
//                .strafeTo(new Vector2d(xF3, yF3))
//                .build();
//
//        // Intake only while driving
//        Action toF3WithIntake = utils.driveWithIntake(toF3TurnEarly, intake, 1.0);
//        Actions.runBlocking(utils.withPoseTelemetry(toF3WithIntake, drive, nShooter, false));
//
//        // Back to shooting pose near F6, then shoot the 3 pickup balls
//        Pose2d poseAtF3 = drive.localizer.getPose();
//
//        Action poseBackToShootingPoseFromF3 = utils.GoToPose(poseAtF3, drive, shootingPoseNear); //utils.GoToPose(fromPose, drive, toPose) Go to from currentpose to a new topose
//        Action poseBackToShootingPoseFromF3WithVision = utils.driveWithLimelightTx(poseBackToShootingPoseFromF3, drive);
//
//        // Shooter ON only for the “with vision” return + align + shoot
//        nShooter.setEnabled(true);
//        utils.setLimelightSearchOmega(0.0);
//        Actions.runBlocking(utils.withPoseTelemetry(poseBackToShootingPoseFromF3WithVision, drive, nShooter, false));
//
//        utils.setLimelightSearchOmega(-0.25);
//        Actions.runBlocking(utils.withPoseTelemetry(
//                utils.alignToTagTxAndDistanceCm(drive, TARGET_DISTANCE_CM, 2),
//                drive, fShooter, false
//        ));
//
//        Actions.runBlocking(utils.withPoseTelemetry(
//                new SequentialAction(
//                        burstF3
//                ),
//                drive, nShooter, true
//        ));
//
//        // ============================================================
//        // PART 5 (RED): ShootingPose (current pose) -> F2 (relative tiles)
//        // One trajectory: rotate to intake-facing-F while moving
//        // No separate .turn() step
//        // ============================================================
//
//        // Shooter OFF during intake + travel
//        nShooter.setEnabled(false);
//
//        // Your Part5 target (example: D4 -> F2)
//        double rowsDownF2    = 3.2;   // +X
//        double colsTowardF2 = 1.75;   // +Y
//
//        // We want the robot to already be turned by E2 (midpoint):
//        double colsTowardE2Mid = 1.0;  // D -> E
//
//        // Convert to inches
//        double dxF2 = rowsDownF2 * tile;
//        double dyF2 = colsTowardF2 * tile;
//        double dyMidE2 = colsTowardE2Mid * tile;
//
//        // Target positions (LOCAL / relative)
//        double xE2 = shootingPoseNear.position.x + dxF2;
//        double yE2 = shootingPoseNear.position.y + dyMidE2;
//
//        double xF2 = shootingPoseNear.position.x + dxF2;
//        double yF2 = shootingPoseNear.position.y + dyF2;
//
//        // Desired intake heading toward F (+Y)
//        double intakeHeadingF2 = Math.toRadians(90.0);
//
//        // Tangent into the midpoint (direction of travel start->E2)
//        double tangentToE2 = Math.atan2(yE2 - shootingPoseNear.position.y, xE2 - shootingPoseNear.position.x);
//
//        // Build ONE trajectory:
//        //  1) spline to "E2" while rotating to intakeHeading (turn happens early)
//        //  2) strafe straight to F2 with heading already correct
//        Pose2d poseAtE2 = new Pose2d(xE2, yE2, intakeHeadingF2);
//
//        Action toF2TurnEarly = drive.actionBuilder(shootingPoseNear)
//                .splineToLinearHeading(poseAtE2, tangentToE2)
//                .strafeTo(new Vector2d(xF2, yF2))
//                .build();
//
//        // Intake only while driving
//        Action toF2WithIntake = utils.driveWithIntake(toF2TurnEarly, intake, 1.0);
//        Actions.runBlocking(utils.withPoseTelemetry(toF2WithIntake, drive, nShooter, false));
//
//        // Back to shooting pose near F6, then shoot the 3 pickup balls
//        Pose2d poseAtF2 = drive.localizer.getPose();
//        Pose2d shootingPoseFar = new Pose2d(
//                115.0493,          // X
//                -25.0,            // Y
//                Math.toRadians(-25)  // Heading: intake downfield, flywheel upfield toward D6
//        );
//        Action poseToStartPoseAsShootingPoseFromF2 = utils.GoToPose(poseAtF2, drive, shootingPoseFar);
//
//        // Blend Limelight yaw correction while driving back
//        Action poseToStartPoseAsShootingPoseFromF2WithVision =
//                utils.driveWithLimelightTx(poseToStartPoseAsShootingPoseFromF2, drive);
//
//        // Shooter ON only for the “with vision” return + align + shoot
//        fShooter.setEnabled(true);
//        utils.setLimelightSearchOmega(0.0);
//        Actions.runBlocking(utils.withPoseTelemetry(
//                poseToStartPoseAsShootingPoseFromF2WithVision, drive, fShooter, false
//        ));
//
//        utils.setLimelightSearchOmega(-0.25);
//        Actions.runBlocking(utils.withPoseTelemetry(
//                utils.alignToTagTxAndDistanceCm(drive, TARGET_DISTANCE_CM, 2),
//                drive, fShooter, false
//        ));
//
//        Actions.runBlocking(utils.withPoseTelemetry(
//                new SequentialAction(
//                        burstF2
//                ),
//                drive, fShooter, true
//        ));

    }

}
