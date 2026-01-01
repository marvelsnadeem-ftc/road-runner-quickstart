package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.autos.FieldCoordinates;
import org.firstinspires.ftc.teamcode.autos.TileMoveHelper;

@Autonomous(name = "Shockwave Auton Blue Near", group = "Auto")
public class ShockwaveRoadrunnerAutonBlue extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();

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

        // Build intake-only actions
        Action intakeBall1 = intake.intakeForwardForTime(2.0, 1.0);
        Action intakeBall2 = intake.intakeForwardForTime(2.0, 1.0);
        Action intakeBall3 = intake.intakeForwardForTime(2.0, 1.0);
        Action intakeReverse1 = intake.intakeForwardForTime(0.2, -1.0);
        Action intakeReverse2 = intake.intakeForwardForTime(0.2, -1.0);
        Action intakeReverse3 = intake.intakeForwardForTime(0.2, -1.0);

        Action intakeBall4 = intake.intakeForwardForTime(2.0, 1.0);
        Action intakeBall5 = intake.intakeForwardForTime(2.0, 1.0);
        Action intakeBall6 = intake.intakeForwardForTime(2.0, 1.0);
        Action intakeReverse4 = intake.intakeForwardForTime(0.2, -1.0);
        Action intakeReverse5 = intake.intakeForwardForTime(0.2, -1.0);
        Action intakeReverse6 = intake.intakeForwardForTime(0.2, -1.0);

        Action intakeBall7 = intake.intakeForwardForTime(2.0, 1.0);
        Action intakeBall8 = intake.intakeForwardForTime(2.0, 1.0);
        Action intakeBall9 = intake.intakeForwardForTime(2.0, 1.0);
        Action intakeReverse7 = intake.intakeForwardForTime(0.2, -1.0);
        Action intakeReverse8 = intake.intakeForwardForTime(0.2, -1.0);
        Action intakeReverse9 = intake.intakeForwardForTime(0.2, -1.0);

        Action intakeBall10 = intake.intakeForwardForTime(2.0, 1.0);
        Action intakeBall11 = intake.intakeForwardForTime(2.0, 1.0);
        Action intakeBall12 = intake.intakeForwardForTime(2.0, 1.0);
        Action intakeReverse10 = intake.intakeForwardForTime(0.2, -1.0);
        Action intakeReverse11 = intake.intakeForwardForTime(0.2, -1.0);
        Action intakeReverse12 = intake.intakeForwardForTime(0.2, -1.0);

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

        // ShootingPose => trapdoor allowed to open if RPM is in range
        Actions.runBlocking(withPoseTelemetry(new SequentialAction(
                        intakeReverse1,
                        intakeBall1,
                        intakeReverse2,
                        intakeBall2,
                        intakeReverse3,
                        intakeBall3),
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

        Action part2MoveIntakeParallel = new ParallelAction(
                strafetoA3,
                intake.intakeForwardForTime(2.0, 1.0)
        );

        // Moving => trapdoor closed
        Actions.runBlocking(withPoseTelemetry(part2MoveIntakeParallel, drive, shooter, false));

        // Back to shootingPose, then shoot sequence (trapdoor allowed only during shooting sequence)
        Pose2d poseAfterStep2 = drive.localizer.getPose();
        Action poseBackToShootingPose2 = ShootingPose(shootingPoseNear, drive, poseAfterStep2);

        Actions.runBlocking(withPoseTelemetry(poseBackToShootingPose2, drive, shooter, false));

        Actions.runBlocking(withPoseTelemetry(
                new SequentialAction(
                        intakeReverse4,
                        intakeBall4,
                        intakeReverse5,
                        intakeBall5,
                        intakeReverse6,
                        intakeBall6
                ),
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
                .splineToLinearHeading(a2IntakePose, travelTangent_A2)
                .build();

        Action part3MoveIntakeParallel = new ParallelAction(
                splineToA2,
                intake.intakeForwardForTime(4.0, 1.0)
        );

        Actions.runBlocking(withPoseTelemetry(part3MoveIntakeParallel, drive, shooter, false));

        Pose2d AfterA2 = drive.localizer.getPose();
        Action poseBackToShootingPoseA2 = ShootingPose(shootingPoseNear, drive, AfterA2);

        Actions.runBlocking(withPoseTelemetry(poseBackToShootingPoseA2, drive, shooter, false));

        Actions.runBlocking(withPoseTelemetry(
                new SequentialAction(
                        intakeReverse7,
                        intakeBall7,
                        intakeReverse8,
                        intakeBall8,
                        intakeReverse9,
                        intakeBall9
                ),
                drive, shooter, true
        ));

        // PART 4: pickup A1, return, shoot
        Action turnMinus45FromShoot2 = drive.actionBuilder(drive.localizer.getPose())
                .turn(Math.toRadians(-45))
                .build();
        Actions.runBlocking(withPoseTelemetry(turnMinus45FromShoot2, drive, shooter, false));

        poseAfterTurn = drive.localizer.getPose();

        double rowsDown = -1.0;
        double deltaFieldForwardDown = rowsDown * tile;

        double deltaRobotXDown = TileMoveHelper.X_SIGN * deltaFieldForwardDown;
        double targetX_C3 = poseAfterTurn.position.x + deltaRobotXDown;

        Action moveToC3 = drive.actionBuilder(poseAfterTurn)
                .lineToX(targetX_C3)
                .build();

        Actions.runBlocking(withPoseTelemetry(moveToC3, drive, shooter, false));

        Pose2d poseAtC2 = drive.localizer.getPose();

        double deltaRowsToA1   = -0.8 * tile;
        double deltaLeftToA1   =  1.2 * tile;

        double deltaX_A1 = TileMoveHelper.X_SIGN * deltaRowsToA1;
        double deltaY_A1 = TileMoveHelper.Y_SIGN * deltaLeftToA1;

        double targetX_A1 = poseAtC2.position.x + deltaX_A1;
        double targetY_A1 = poseAtC2.position.y + deltaY_A1;

        double travelTangent_A1 = Math.atan2(
                targetY_A1 - poseAtC2.position.y,
                targetX_A1 - poseAtC2.position.x
        );

        double headingOffset_A1 = Math.toRadians(-35);
        double finalHeading_A1  = travelTangent_A1 + headingOffset_A1;

        Pose2d a1IntakePose = new Pose2d(
                targetX_A1,
                targetY_A1,
                finalHeading_A1
        );

        Action splineToA1 = drive.actionBuilder(poseAtC3)
                .splineToLinearHeading(a1IntakePose, travelTangent_A1)
                .build();

        Action part4MoveIntakeParallel = new ParallelAction(
                splineToA1,
                intake.intakeForwardForTime(5.0, 1.0)
        );

        Actions.runBlocking(withPoseTelemetry(part4MoveIntakeParallel, drive, shooter, false));

        Pose2d poseAfterA1 = drive.localizer.getPose();
        Action poseBackToShootingPoseA1 = ShootingPose(shootingPoseNear, drive, poseAfterA1);

        Actions.runBlocking(withPoseTelemetry(poseBackToShootingPoseA1, drive, shooter, false));

        Actions.runBlocking(withPoseTelemetry(
                new SequentialAction(
                        intakeReverse10,
                        intakeBall10,
                        intakeReverse11,
                        intakeBall11,
                        intakeReverse12,
                        intakeBall12
                ),
                drive, shooter, true
        ));
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