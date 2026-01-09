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

@Autonomous(name = "Shockwave Auton Far Blue", group = "Auto")
public class SWAutonBlueFar extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();
    // === TIMING CONSTANTS (tune these on field) ===
    private static final double INTAKE_FORWARD_SEC  = 1.0;   // was 2.0
    private static final double INTAKE_REVERSE_SEC  = 0.20;  // was 0.2

    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));

        // Single shooter instance (continuous run)
        Shooter shooter = new Shooter(hardwareMap, 2800);

        Intake intake = new Intake(hardwareMap);

        // Shooter removed from ShootAndIntakeAction (intake-only actions now)
//        ShootAndIntakeAction intakeShoot = new ShootAndIntakeAction(intake, 2.0, 1.0);
//        ShootAndIntakeAction intakeShootReverse = new ShootAndIntakeAction(intake, 0.2, -1.0);

        // Show live pose on INIT so you can sanity check Pinpoint signs.
        while (!isStopRequested() && !opModeIsActive()) {
            Pose2d p = drive.localizer.getPose();
            telemetry.addLine("INIT Pose (from Pinpoint / RR):");
            telemetry.addData("X", p.position.x);
            telemetry.addData("Y", p.position.y);
            telemetry.addData("Heading (deg)", Math.toDegrees(p.heading.toDouble()));
            telemetry.update();
        }

        TileMoveHelper.X_SIGN = -1.0;
        TileMoveHelper.Y_SIGN = -1.0;

        Pose2d shootingPoseNear;

        waitForStart();
        if (isStopRequested()) return;
        runtime.reset();

        // Build bursts as needed (fresh action each time)
        Action burstPreload = buildIntakeBurst(intake, 3);
        Action burstA3      = buildIntakeBurst(intake, 3);
        Action burstA2      = buildIntakeBurst(intake, 3);
        Action burstA1      = buildIntakeBurst(intake, 3);

        // PART 1 Go and Shoot
        Pose2d startPose = drive.localizer.getPose(); // flywheel facing C6 (up)

        double deltaFieldForward = 2.75 * TileMoveHelper.TILE;
        double deltaRobotX       = TileMoveHelper.X_SIGN * deltaFieldForward;
        double targetX           = startPose.position.x + deltaRobotX;

        Action moveRows3TurnAndShootPose = drive.actionBuilder(startPose)
                .lineToX(targetX)
                .turn(Math.toRadians(35))
                .build();

        // Moving => trapdoor must be closed
        Actions.runBlocking(withPoseTelemetry(moveRows3TurnAndShootPose, drive, shooter, false));

        // Preload shoot (was intakeReverse1/2/3 + intakeBall1/2/3)
        Actions.runBlocking(withPoseTelemetry(
                burstPreload,
                drive, shooter, true
        ));

        shootingPoseNear = drive.localizer.getPose();
        telemetry.addLine("Shooting pose recorded (C4-ish)");
        telemetry.addData("Shoot X", shootingPoseNear.position.x);
        telemetry.addData("Shoot Y", shootingPoseNear.position.y);
        telemetry.addData("Shoot H (deg)", Math.toDegrees(shootingPoseNear.heading.toDouble()));
        telemetry.update();

        // PART 2: rotate intake, move to A3 while intaking
        Action rotateIntakeToA3 = drive.actionBuilder(shootingPoseNear)
                .turn(Math.toRadians(-125))
                .build();

        Actions.runBlocking(withPoseTelemetry(rotateIntakeToA3, drive, shooter, false));

        Pose2d poseAfterTurn = drive.localizer.getPose();
        double tile = FieldCoordinates.TILE;

        double deltaRows = 1.5 * tile;
        double colsLeft = 1.20 * tile;

        double flywheelHeading = poseAfterTurn.heading.toDouble();
        double intakeHeading   = flywheelHeading + Math.PI;

        double forwardX = poseAfterTurn.position.x + TileMoveHelper.X_SIGN * deltaRows * Math.cos(intakeHeading);
        double forwardY = poseAfterTurn.position.y + TileMoveHelper.Y_SIGN * colsLeft * Math.sin(intakeHeading);

        Action strafetoA3 = drive.actionBuilder(poseAfterTurn)
                .strafeTo(new Vector2d(forwardX, forwardY))
                .build();

        Action part2DriveWithIntake = driveWithIntake(strafetoA3, intake, 1.0);
        // Moving => trapdoor closed
        Actions.runBlocking(withPoseTelemetry(part2DriveWithIntake, drive, shooter, false));

        // Back to shootingPose, then shoot sequence (trapdoor allowed only during shooting sequence)
        Pose2d poseAtA3 = drive.localizer.getPose();
        Action poseBackToShootingPoseFromA2 = ShootingPose(shootingPoseNear, drive, poseAtA3);

        Actions.runBlocking(withPoseTelemetry(poseBackToShootingPoseFromA2, drive, shooter, false));

        // A3 shoot
        Actions.runBlocking(withPoseTelemetry(
                burstA3,
                drive, shooter, true
        ));

        // PART 3: Turn -45, spline to A2 while intaking, return and shoot
        Action turnMinus45FromShoot = drive.actionBuilder(shootingPoseNear)
                .turn(Math.toRadians(-45))
                .build();
        Actions.runBlocking(withPoseTelemetry(turnMinus45FromShoot, drive, shooter, false));

        Pose2d poseAtC3 = drive.localizer.getPose();

        double deltaRowsToA2   = -1 * tile;
        double deltaLeftToA2   =  1.2 * tile;

        double deltaX_A2 = TileMoveHelper.X_SIGN * deltaRowsToA2;
        double deltaY_A2 = TileMoveHelper.Y_SIGN * deltaLeftToA2;

        double targetX_A2 = poseAtC3.position.x + deltaX_A2;
        double targetY_A2 = poseAtC3.position.y + deltaY_A2;

        double travelTangent_A2 = Math.atan2(
                targetY_A2 - poseAtC3.position.y,
                targetX_A2 - poseAtC3.position.x
        );

        double headingOffset_A2 = Math.toRadians(-38);
        double finalHeading_A2  = travelTangent_A2 + headingOffset_A2;

        Pose2d a2IntakePose = new Pose2d(
                targetX_A2,
                targetY_A2,
                finalHeading_A2
        );

        Action splineToA2 = drive.actionBuilder(poseAtC3)
                .splineToLinearHeading(
                        a2IntakePose,
                        travelTangent_A2
                )
                .build();

        // Intake runs only while splineToA2 is executing
        Action part3DriveWithIntake = driveWithIntake(splineToA2, intake, 1.0);
        Actions.runBlocking(withPoseTelemetry(part3DriveWithIntake, drive, shooter, false));


        Pose2d AfterA2 = drive.localizer.getPose();
        Action poseBackToShootingPoseA2 = ShootingPose(shootingPoseNear, drive, AfterA2);

        Actions.runBlocking(withPoseTelemetry(poseBackToShootingPoseA2, drive, shooter, false));


        // A2 shoot
        Actions.runBlocking(withPoseTelemetry(
                burstA2,
                drive, shooter, true
        ));


        // ===== PART 4: pickup A1, return, shoot =====

// Read the starting pose for Part 4 (C4 shooting pose)
        Pose2d part4Start = drive.localizer.getPose();

// Row/column semantics (you already tuned these)
        double rowsDown    = -1.0;    // C4 -> C3/C2-style drop
        double deltaRowsToA1 = -0.8;  // extra down toward A1
        double deltaLeftToA1 =  1.2;  // left toward A1

// 1) Compute C2 row target in world coords from the Part4 start pose
        double deltaFieldForwardDown = rowsDown * tile;
        double deltaRobotXDown       = TileMoveHelper.X_SIGN * deltaFieldForwardDown;

        double c2X = part4Start.position.x + deltaRobotXDown;
        double c2Y = part4Start.position.y; // same lateral lane

// 2) Compute A1 intake target from that C2 position
        double deltaX_A1 = TileMoveHelper.X_SIGN * (deltaRowsToA1 * tile);
        double deltaY_A1 = TileMoveHelper.Y_SIGN * (deltaLeftToA1 * tile);

        double a1X = c2X + deltaX_A1;
        double a1Y = c2Y + deltaY_A1;

// 3) Direction of travel from C2 -> A1
        double travelTangent_A1 = Math.atan2(
                a1Y - c2Y,
                a1X - c2X
        );

// 4) Final heading at A1 (intake orientation tweak)
        double headingOffset_A1 = Math.toRadians(-35);  // your tuned value
        double finalHeading_A1  = travelTangent_A1 + headingOffset_A1;

        Pose2d a1IntakePose = new Pose2d(
                a1X,
                a1Y,
                finalHeading_A1
        );

// 5) Build ONE smooth trajectory: turn -> move down 1 row -> spline to A1
        Action part4Path = drive.actionBuilder(part4Start)
                .turn(Math.toRadians(-45))      // STEP 1: turn from shooting pose
                .lineToX(c2X)                   // STEP 2: drop one row to C2 lane
                .splineToLinearHeading(         // STEP 3: curve into A1 intake pose
                        a1IntakePose,
                        travelTangent_A1
                )
                .build();

// Intake runs only while this whole A1 path executes
        Action part4DriveWithIntake = driveWithIntake(part4Path, intake, 1.0);

// Single runBlocking for all A1 movement
        Actions.runBlocking(withPoseTelemetry(part4DriveWithIntake, drive, shooter, false));

        //Shooting Logic for A1
        Pose2d poseAfterA1 = drive.localizer.getPose();
        Action poseBackToShootingPoseA1 = ShootingPose(shootingPoseNear, drive, poseAfterA1);

        Actions.runBlocking(withPoseTelemetry(poseBackToShootingPoseA1, drive, shooter, false));
        // A1 shoot
        Actions.runBlocking(withPoseTelemetry(
                burstA1,
                drive, shooter, true
        ));

    }

    private Action buildIntakeBurst(Intake intake, int numBalls) {
        Action[] steps = new Action[numBalls * 2];
        for (int i = 0; i < numBalls; i++) {
            // reverse a bit
            steps[2 * i]     = intake.intakeForwardForTime(INTAKE_REVERSE_SEC, -1.0);
            // then feed forward
            steps[2 * i + 1] = intake.intakeForwardForTime(INTAKE_FORWARD_SEC,  1.0);
        }
        return new SequentialAction(steps);
    }


    // Generic helper: from any currentPose, build an Action that splines back to shootingPose.
    private Action ShootingPose(Pose2d shootingPose, MecanumDrive drive, Pose2d currentPose) {
        if (shootingPose == null) {
            throw new IllegalStateException("shootingPose is not set before calling buildSplineBackToShootingPose()");
        }

        double travelTangent = Math.atan2(
                shootingPose.position.y - currentPose.position.y,
                shootingPose.position.x - currentPose.position.x
        );

        return drive.actionBuilder(currentPose)
                .splineToLinearHeading(shootingPose, travelTangent)
                .build();
    }

    private Action driveWithIntake(Action driveAction, Intake intake, double power) {
        return new Action() {
            private boolean started = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!started) {
                    // Start intake once, when the action begins
                    intake.setPower(power);
                    started = true;
                }

                // Step the drive action
                boolean driveStillRunning = driveAction.run(packet);

                if (!driveStillRunning) {
                    // Drive finished -> stop intake and end this wrapper action
                    intake.setPower(0.0);
                }

                // This wrapper lives exactly as long as the drive
                return driveStillRunning;
            }
        };
    }

    private Action withPoseTelemetry(Action inner, MecanumDrive drive, Shooter shooter, boolean allowTrapdoor) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // Continuous shooter update; trapdoor forced closed while moving
                shooter.update(allowTrapdoor);

                // Telemetry
                Pose2d pose = drive.localizer.getPose();

                packet.put("X", pose.position.x);
                packet.put("Y", pose.position.y);
                packet.put("HeadingDeg", Math.toDegrees(pose.heading.toDouble()));
                packet.put("Run Time", runtime.toString());
                packet.put("Target RPM", shooter.getBangBangTargetVelocity());
                packet.put("Actual RPM", shooter.getActualRPM());
                packet.put("TrapdoorAllowed", allowTrapdoor);

                telemetry.addData("X", pose.position.x);
                telemetry.addData("Y", pose.position.y);
                telemetry.addData("Heading (deg)", Math.toDegrees(pose.heading.toDouble()));
                telemetry.addData("Run Time", runtime.toString());
                telemetry.addData("Target RPM", shooter.getBangBangTargetVelocity());
                telemetry.addData("Actual RPM", shooter.getActualRPM());
                telemetry.addData("TrapdoorAllowed", allowTrapdoor);
                telemetry.update();

                return inner.run(packet);
            }
        };
    }
}
