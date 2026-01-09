package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.autos.FieldCoordinates;
import org.firstinspires.ftc.teamcode.autos.TileMoveHelper;

@Autonomous(name = "Shockwave Auton Near Goal Red", group = "Auto")
public class SWAutonRedNearGoal extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();

    // === TIMING CONSTANTS (same as Blue, tune on field if needed) ===
    private static final double INTAKE_FORWARD_SEC  = 1.0;
    private static final double INTAKE_REVERSE_SEC  = 0.20;

    @Override
    public void runOpMode() throws InterruptedException {

        // Seed pose; Pinpoint / RR localizer will give us the live pose.
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        // Shooter runs continuously with bang-bang, same as Blue
        Shooter nShooter = new Shooter(hardwareMap, 2800);
        Shooter fShooter = new Shooter(hardwareMap, 3500);
        Intake  intake  = new Intake(hardwareMap);

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

        Pose2d shootingPoseNear;

        waitForStart();
        if (isStopRequested()) return;

        runtime.reset();

        // Build bursts as needed (fresh action each time)
        Action burstPreload = buildIntakeBurst(intake, 3);
        Action burstF5      = buildIntakeBurst(intake, 3);
        Action burstF4      = buildIntakeBurst(intake, 3);
        Action burstF3      = buildIntakeBurst(intake, 3);
        Action burstF2      = buildIntakeBurst(intake, 3);

        // ============================================================
        // PART 1 (RED NEAR): Start at E6 (intake facing E1, flywheel facing E6 boundary)
        // Move to D4 intake-forward, turn right toward F6, shoot preloads.
        // This becomes shootingPoseNear (flywheel facing goal).
        // ============================================================

        Pose2d startPose = drive.localizer.getPose(); // robot placed at E6

        // E6 -> D4 is:
        //   +2 tiles in X (row 6 -> row 4)
        //   -1 tile  in Y (col E -> col D)
        double xD4 = startPose.position.x + (1.5 * FieldCoordinates.TILE);
        double yD4 = startPose.position.y + (-1.0 * FieldCoordinates.TILE);

        // Keep the same "turn right toward F6" magnitude you already validated in the attached logic.
        Action moveToD4TurnAndShootPose = drive.actionBuilder(startPose)
                .strafeTo(new Vector2d(xD4, yD4))
                .turn(Math.toRadians(-40))    // right turn toward F6
                .build();

        Actions.runBlocking(withPoseTelemetry(moveToD4TurnAndShootPose, drive, nShooter, false));

        // Shoot 3 preloads with intake-only burst (shooter already running)
//        Actions.runBlocking(withPoseTelemetry(
//                burstPreload,
//                drive, nShooter, true
//        ));

        shootingPoseNear = drive.localizer.getPose();

        telemetry.addLine("Red shooting pose recorded (D4-ish pointing F6)");
        telemetry.addData("Shoot X", shootingPoseNear.position.x);
        telemetry.addData("Shoot Y", shootingPoseNear.position.y);
        telemetry.addData("Shoot H (deg)", Math.toDegrees(shootingPoseNear.heading.toDouble()));
        telemetry.update();

        // ============================================================
        // PART 2: Rotate robot towards D5 intake facing F5 from ShootingPose, drive and intake in artifacts from F5, return and shoot
        //
        // ============================================================

        Action rotateIntakeToD5 = drive.actionBuilder(shootingPoseNear)
                .turn(Math.toRadians(115))
                .build();

        Actions.runBlocking(withPoseTelemetry(rotateIntakeToD5, drive, nShooter, false));


        double tile = FieldCoordinates.TILE;
        double tilesForwardToF5 = -1.75;  // D5 -> F5 is 2 tiles

        Pose2d poseAtD5 = drive.localizer.getPose();   // near D5

        double targetY_F5 = poseAtD5.position.y - tilesForwardToF5 * tile;  // -12 - 48 = -60

        Action toF5Center = drive.actionBuilder(poseAtD5)
                .strafeTo(new Vector2d(poseAtD5.position.x, targetY_F5))
                .build();

        // With intake running while driving
        Action driveToF5WithIntake = driveWithIntake(toF5Center, intake, 1.0);
        Actions.runBlocking(withPoseTelemetry(driveToF5WithIntake, drive, nShooter, false));


        // Back to shooting pose near F6, then shoot the 3 pickup balls
        Pose2d poseAtF5 = drive.localizer.getPose();
        Action poseBackToShootingPoseFromF5 = GoToPose(poseAtF5, drive, shootingPoseNear); //GoToPose(fromPose, drive, toPose) Go to from currentpose to a new topose

        Actions.runBlocking(withPoseTelemetry(poseBackToShootingPoseFromF5, drive, nShooter, false));

        Action poseBackToD5FromShootingPose = GoToPose(shootingPoseNear, drive, poseAtD5);
//        Actions.runBlocking(withPoseTelemetry(
//                new SequentialAction(
//                        burstF5
//                        ,poseBackToD5FromShootingPose
//                ),
//                drive, nShooter, true
//        ));


        // ============================================================
        // PART 3 (RED): from D5 intake-facing-F5 -> F4 (one step)
        // Authoritative convention (flywheel-forward map):
        //   upfield (toward row 6) = -X
        //   toward F (column F)    = +Y
        // ============================================================

        // Your Part3 target (example: D4 -> F3)
        double rowsDownF4    = 1.2;   // +X
        double colsTowardF4 = 1.75;   // +Y

        // We want the robot to already be turned by E4 (midpoint):
        double colsTowardE4Mid = 1.0;  // D -> E

        // Convert to inches
        double dxF4 = rowsDownF4 * tile;
        double dyF4 = colsTowardF4 * tile;
        double dyMidE4 = colsTowardE4Mid * tile;

        // Target positions (LOCAL / relative)
        double xE4 = shootingPoseNear.position.x + dxF4;
        double yE4 = shootingPoseNear.position.y + dyMidE4;

        double xF4 = shootingPoseNear.position.x + dxF4;
        double yF4 = shootingPoseNear.position.y + dyF4;

        // Desired intake heading toward F (+Y)
        double intakeHeadingF4 = Math.toRadians(90.0);

        // Tangent into the midpoint (direction of travel start->E4)
        double tangentToE4 = Math.atan2(yE4 - shootingPoseNear.position.y, xE4 - shootingPoseNear.position.x);

        // Build ONE trajectory:
        //  1) spline to "E4" while rotating to intakeHeading (turn happens early)
        //  2) strafe straight to F4 with heading already correct
        Pose2d poseAtE4 = new Pose2d(xE4, yE4, intakeHeadingF4);

        Action toF4TurnEarly = drive.actionBuilder(shootingPoseNear)
                .splineToLinearHeading(poseAtE4, tangentToE4)
                .strafeTo(new Vector2d(xF4, yF4))
                .build();

        // Intake only while driving
        Action toF4WithIntake = driveWithIntake(toF4TurnEarly, intake, 1.0);
        Actions.runBlocking(withPoseTelemetry(toF4WithIntake, drive, nShooter, false));

        // Back to shooting pose near F6, then shoot the 3 pickup balls
        Pose2d poseAtF4 = drive.localizer.getPose();
        Action poseBackToShootingPoseFromF4 = GoToPose(poseAtF4, drive, shootingPoseNear); //GoToPose(fromPose, drive, toPose) Go to from currentpose to a new topose

        Actions.runBlocking(withPoseTelemetry(poseBackToShootingPoseFromF4, drive, nShooter, false));

//        Actions.runBlocking(withPoseTelemetry(
//                new SequentialAction(
//                        burstF4
//                ),
//                drive, nShooter, true
//        ));


        // ============================================================
        // PART 4 (RED): ShootingPose (current pose) -> F3 (relative tiles)
        // One trajectory: rotate to intake-facing-F while moving
        // No separate .turn() step
        // ============================================================

        // Your Part4 target (example: D4 -> F3)
        double rowsDownF3    = 2.2;   // +X
        double colsTowardF3 = 1.75;   // +Y

        // We want the robot to already be turned by E3 (midpoint):
        double colsTowardE3Mid = 1.0;  // D -> E

        // Convert to inches
        double dxF3 = rowsDownF3 * tile;
        double dyF3 = colsTowardF3 * tile;
        double dyMidE3 = colsTowardE3Mid * tile;

        // Target positions (LOCAL / relative)
        double xE3 = shootingPoseNear.position.x + dxF3;
        double yE3 = shootingPoseNear.position.y + dyMidE3;

        double xF3 = shootingPoseNear.position.x + dxF3;
        double yF3 = shootingPoseNear.position.y + dyF3;

        // Desired intake heading toward F (+Y)
        double intakeHeadingF3 = Math.toRadians(90.0);

        // Tangent into the midpoint (direction of travel start->E3)
        double tangentToE3 = Math.atan2(yE3 - shootingPoseNear.position.y, xE3 - shootingPoseNear.position.x);

        // Build ONE trajectory:
        //  1) spline to "E3" while rotating to intakeHeading (turn happens early)
        //  2) strafe straight to F3 with heading already correct
        Pose2d poseAtE3 = new Pose2d(xE3, yE3, intakeHeadingF3);

        Action toF3TurnEarly = drive.actionBuilder(shootingPoseNear)
                .splineToLinearHeading(poseAtE3, tangentToE3)
                .strafeTo(new Vector2d(xF3, yF3))
                .build();

        // Intake only while driving
        Action toF3WithIntake = driveWithIntake(toF3TurnEarly, intake, 1.0);
        Actions.runBlocking(withPoseTelemetry(toF3WithIntake, drive, nShooter, false));

        // Back to shooting pose near F6, then shoot the 3 pickup balls
        Pose2d poseAtF3 = drive.localizer.getPose();
        Action poseBackToShootingPoseFromF3 = GoToPose(poseAtF3, drive, shootingPoseNear); //GoToPose(fromPose, drive, toPose) Go to from currentpose to a new topose

        Actions.runBlocking(withPoseTelemetry(poseBackToShootingPoseFromF3, drive, nShooter, false));

        //Action poseBackToD5FromShootingPoseAfterF3 = GoToPose(shootingPoseNear, drive, poseAtD5);
//        Actions.runBlocking(withPoseTelemetry(
//                new SequentialAction(
//                        burstF3
//                        //,poseBackToD5FromShootingPoseAfterF3
//                ),
//                drive, nShooter, true
//        ));
//
        // ============================================================
        // PART 5 (RED): ShootingPose (current pose) -> F2 (relative tiles)
        // One trajectory: rotate to intake-facing-F while moving
        // No separate .turn() step
        // ============================================================

        // Your Part5 target (example: D4 -> F2)
        double rowsDownF2    = 3.2;   // +X
        double colsTowardF2 = 1.75;   // +Y

        // We want the robot to already be turned by E2 (midpoint):
        double colsTowardE2Mid = 1.0;  // D -> E

        // Convert to inches
        double dxF2 = rowsDownF2 * tile;
        double dyF2 = colsTowardF2 * tile;
        double dyMidE2 = colsTowardE2Mid * tile;

        // Target positions (LOCAL / relative)
        double xE2 = shootingPoseNear.position.x + dxF2;
        double yE2 = shootingPoseNear.position.y + dyMidE2;

        double xF2 = shootingPoseNear.position.x + dxF2;
        double yF2 = shootingPoseNear.position.y + dyF2;

        // Desired intake heading toward F (+Y)
        double intakeHeadingF2 = Math.toRadians(90.0);

        // Tangent into the midpoint (direction of travel start->E2)
        double tangentToE2 = Math.atan2(yE2 - shootingPoseNear.position.y, xE2 - shootingPoseNear.position.x);

        // Build ONE trajectory:
        //  1) spline to "E2" while rotating to intakeHeading (turn happens early)
        //  2) strafe straight to F2 with heading already correct
        Pose2d poseAtE2 = new Pose2d(xE2, yE2, intakeHeadingF2);

        Action toF2TurnEarly = drive.actionBuilder(shootingPoseNear)
                .splineToLinearHeading(poseAtE2, tangentToE2)
                .strafeTo(new Vector2d(xF2, yF2))
                .build();

        // Intake only while driving
        Action toF2WithIntake = driveWithIntake(toF2TurnEarly, intake, 1.0);
        Actions.runBlocking(withPoseTelemetry(toF2WithIntake, drive, nShooter, false));

        // PART 5 (RED) add-on: from F2 (after pickup, intake-facing) -> D1, arrive with FLYWHEEL facing D6

        //Pose2d poseAtF2 = drive.localizer.getPose();

//// D1 tile center is (X=+60, Y=+12) == (2.5 tiles, 0.5 tiles)
//        double xD1 = 2.5 * tile;
//        double yD1 = 0.5 * tile;
//
//// Flywheel facing D6 means pointing "upfield" (-X) at D1 => heading = 180 deg
//        Pose2d poseAtD1FlywheelToD6 = new Pose2d(xD1, yD1, Math.toRadians(180.0));
//
//        double tangentToD1 = Math.atan2(
//                poseAtD1FlywheelToD6.position.y - poseAtF2.position.y,
//                poseAtD1FlywheelToD6.position.x - poseAtF2.position.x
//        );
//
//        Action poseBackToD1FromF2_FlywheelToD6 = drive.actionBuilder(poseAtF2)
//                .splineToLinearHeading(poseAtD1FlywheelToD6, tangentToD1)
//                .build();
//
//        Actions.runBlocking(withPoseTelemetry(poseBackToD1FromF2_FlywheelToD6, drive, fShooter, false));

        // Back to start pose near E6, then shoot the 3 pickup balls
        Pose2d poseAtF2 = drive.localizer.getPose();
        //Pose2d shootingPoseFar = new Pose2d(110.7995, 25, Math.toRadians(25));
        Pose2d shootingPoseFar = new Pose2d(
                115.0493,          // X
                -25.0,            // Y
                Math.toRadians(-25)  // Heading: intake downfield, flywheel upfield toward D6
        );

        Action poseBackToShootingPoseFromF2 = GoToPose(poseAtF2, drive, shootingPoseFar); //GoToPose(fromPose, drive, toPose) Go to from currentpose to a new topose

        Actions.runBlocking(withPoseTelemetry(poseBackToShootingPoseFromF2, drive, nShooter, false));

        //  .turn(-42) so we rotate toward F6 (right goal)
//        Action goF2toD1AndFarShootPose = drive.actionBuilder(startPose)
//                .turn(Math.toRadians(-20))    // turn right toward F6 instead of left toward A6
//                .build();
//
//        Actions.runBlocking(withPoseTelemetry(goF2toD1AndFarShootPose, drive, fShooter, false));
//
////        Actions.runBlocking(withPoseTelemetry(
////                new SequentialAction(
////                        burstF2
////                ),
////                drive, fShooter, true
////        ));
//
//        Pose2d shootingPoseFar = drive.localizer.getPose();
    }

    // ==== Helpers =====================================

    private Action buildIntakeBurst(Intake intake, int numBalls) {
        Action[] steps = new Action[numBalls * 2];
        for (int i = 0; i < numBalls; i++) {
            steps[2 * i]     = intake.intakeForwardForTime(INTAKE_REVERSE_SEC, -1.0);
            steps[2 * i + 1] = intake.intakeForwardForTime(INTAKE_FORWARD_SEC,  1.0);
        }
        return new SequentialAction(steps);
    }

    // Generic helper: from any currentPose, build an Action that splines back to any other Pose.
    private Action GoToPose(Pose2d fromPose, MecanumDrive drive, Pose2d toPose) {
        if (toPose == null) {
            throw new IllegalStateException("shootingPose is not set before calling ShootingPose()");
        }

        double travelTangent = Math.atan2(
                toPose.position.y - fromPose.position.y,
                toPose.position.x - fromPose.position.x
        );

        return drive.actionBuilder(fromPose)
                .splineToLinearHeading(toPose, travelTangent)
                .build();
    }

    private Action driveWithIntake(Action driveAction, Intake intake, double power) {
        return new Action() {
            private boolean started = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!started) {
                    intake.setPower(power);
                    started = true;
                }

                boolean driveStillRunning = driveAction.run(packet);

                if (!driveStillRunning) {
                    intake.setPower(0.0);
                }

                return driveStillRunning;
            }
        };
    }

    private Action withPoseTelemetry(Action inner, MecanumDrive drive, Shooter shooter, boolean trapdoorOpen) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // Continuous shooter update
                shooter.update(trapdoorOpen);

                Pose2d pose = drive.localizer.getPose();

                packet.put("X", pose.position.x);
                packet.put("Y", pose.position.y);
                packet.put("HeadingDeg", Math.toDegrees(pose.heading.toDouble()));
                packet.put("Run Time", runtime.toString());
                packet.put("Target RPM", shooter.getBangBangTargetVelocity());
                packet.put("Actual RPM", shooter.getActualRPM());
                packet.put("Trapdoor", trapdoorOpen ? "OPEN" : "CLOSED");

                telemetry.addData("X", pose.position.x);
                telemetry.addData("Y", pose.position.y);
                telemetry.addData("Heading (deg)", Math.toDegrees(pose.heading.toDouble()));
                telemetry.addData("Run Time", runtime.toString());
                telemetry.addData("Target RPM", shooter.getBangBangTargetVelocity());
                telemetry.addData("Actual RPM", shooter.getActualRPM());
                telemetry.addData("Trapdoor", trapdoorOpen ? "OPEN" : "CLOSED");
                telemetry.update();

                return inner.run(packet);
            }
        };
    }
}
