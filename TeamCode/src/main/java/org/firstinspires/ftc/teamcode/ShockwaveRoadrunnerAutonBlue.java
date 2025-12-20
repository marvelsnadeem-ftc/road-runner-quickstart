package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Shockwave Auton Blue", group = "Auto")
public class ShockwaveRoadrunnerAutonBlue extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        Shooter shooter     = new Shooter(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        ShootAndIntakeAction intakeShoot = new ShootAndIntakeAction(shooter, intake, 3.0, 1.0);
        ShootAndIntakeAction intakeShoot1 = new ShootAndIntakeAction(shooter, intake, 3.0, 1.0);
        ShootAndIntakeAction intakeShootReverse = new ShootAndIntakeAction(shooter, intake, 0.3, -1.0);
        ShootAndIntakeAction intakeShootReverse1 = new ShootAndIntakeAction(shooter, intake, 1, -1.0);
        ShootAndIntakeAction intakeShootReverseExtraTime = new ShootAndIntakeAction(shooter, intake, 1, -1.0);


        waitForStart();
        if (isStopRequested()) return;
        Pose2d initialPose = FieldCoordinates.BLUE_START_C1;
        // --------------------------------------------------------------------
        //        // SIMPLE SHOOT ACTION:
        //        //
        //        // Just spin the flywheel using bang-bang for a fixed time.
        //        // You can tune this duration on the field.
        //        // --------------------------------------------------------------------
        double SPIN_AND_SHOOT_TIME = 2.0; // seconds; adjust as needed
        Action spinAndShoot = shooter.spinBangBangForTime(SPIN_AND_SHOOT_TIME);
        Action spinAndShootAfterPickup = shooter.spinBangBangForTime(SPIN_AND_SHOOT_TIME);
        Action intakeShootComboBall1 = intakeShoot.buildShootIntake();
        Action intakeShootComboBall2 = intakeShoot.buildShootIntake();
        Action intakeShootComboBall3 = intakeShoot.buildShootIntake();
        Action intakeShootComboReverse1 = intakeShootReverse.buildShootIntake();
        Action intakeShootComboReverse2 = intakeShootReverse.buildShootIntake();
        Action intakeShootComboReverse3 = intakeShootReverse.buildShootIntake();

        Action intakeShootComboBall4 = intakeShoot1.buildShootIntake();
        Action intakeShootComboBall5 = intakeShoot1.buildShootIntake();
        Action intakeShootComboBall6 = intakeShoot1.buildShootIntake();
        Action intakeShootComboReverse4 = intakeShootReverse1.buildShootIntake();
        Action intakeShootComboReverse5 = intakeShootReverse1.buildShootIntake();
        Action intakeShootComboReverse6 = intakeShootReverse1.buildShootIntake();

        Action intakeShootCombo2 = intakeShoot.buildShootIntake();


//        Pose2d startPose = new Pose2d(0, 0, 0);
//        Pose2d shootPose = new Pose2d(54, 0, Math.toRadians(225));
//
//        Action part1Base = drive.actionBuilder(startPose)
//
//                // Optional: drive straight first for stability, then rotate while finishing the approach
//                .lineToX(40)
//
//                // Arrive at (54,0) already rotated to 225° (front heading),
//                // meaning your BACK/shooter is pointing opposite (225°+180°).
//                // endTangent = direction of travel as you arrive. From x=40 to x=54, you approach along +X, so tangent ~ 0 rad.
//                .splineToLinearHeading(shootPose, 0.0)
//
//                // Shoot actions (unchanged)
//                .stopAndAdd(spinAndShoot)
//                .stopAndAdd(intakeShootComboReverse1)
//                .stopAndAdd(intakeShootComboBall1)
//                .stopAndAdd(intakeShootComboReverse2)
//                .stopAndAdd(intakeShootComboBall2)
//                .stopAndAdd(intakeShootComboReverse3)
//                .stopAndAdd(intakeShootComboBall3)
//
////                // Keep your alignment move (still useful)
////                .lineToX(45)
//
//                // Keep for now; later we can remove with a spline into the intake lane
//                .turn(Math.toRadians(-135))
//
//                .build();
//
//        // run part1
//        Action part1 = withPoseTelemetry(part1Base, drive);
//        Actions.runBlocking(part1);

        // ===== POSES (adjust Y values if needed for your field coordinates) =====
        Pose2d startPose = new Pose2d(0, 0, 0);

// Shooter-on-back shooting pose you already use (front heading = 225°)
        Pose2d shootPose = new Pose2d(54, 8, Math.toRadians(225));

//// Your known alignment/staging pose before entering the intake lane
//        Pose2d stagePose = new Pose2d(45, 0, Math.toRadians(225));
//
//// 3rd-row pickup pose (you used lineToY(45) previously). Heading set to intake-forward direction.
//// If your intake wants to face +Y, 90° is typical. Change if needed.
//        Pose2d pickupPose = new Pose2d(45, 45, Math.toRadians(90));

// ===== PART 1 (optimized motion) =====
// Key optimization: replace ".lineToX(54) + .turn(225)" with a single splineToLinearHeading to arrive already aimed.
        Action part1Move = drive.actionBuilder(startPose)

                // Optional straight segment first for stability; helps keep early motion simple
                .lineToX(40)

                // Rotate while driving into the shooting pose (no in-place turn)
                // endTangent = direction of travel as you arrive. Coming from x=40 -> x=54 along +X => tangent 0 rad.
                .splineToLinearHeading(shootPose, 0.0)

                // After shooting, go to the staging/alignment pose at x=45 while keeping heading 225°
                //.lineToX(stagePose.position.x)

                .build();

// Your shooting actions (unchanged)
        Action part1Shoot = new SequentialAction(
                spinAndShoot,
                intakeShootComboReverse1,
                intakeShootComboBall1,
                intakeShootComboReverse2,
                intakeShootComboBall2,
                intakeShootComboReverse3,
                intakeShootComboBall3
        );


// Run Part 1: move then shoot (keeps mechanisms exactly as you have them)
        Actions.runBlocking(withPoseTelemetry(part1Move, drive));
        Actions.runBlocking(withPoseTelemetry(part1Shoot, drive));


        //Pose2d pickupPoseA4 = new Pose2d(-60, 12, Math.toRadians(90));

// ---- NEW: A4 pickup and return ----
        Pose2d afterPart1 = drive.localizer.getPose();

// choose +90 or -90 after you do the sign test
        double turnToFaceA = Math.toRadians(-145);

        Action moveToA3 = drive.actionBuilder(afterPart1)
                .turn(turnToFaceA)
                .lineToX( afterPart1.position.x + 5.0)   // ONLY correct if +X is robot-forward in your RR frame
                .build();

        Action part2ParallelDriveAndIntake = new ParallelAction(
                moveToA3,
                intake.intakeForwardForTime(3.0, 1.0)
        );
        //Action pickupBalls = intake.intakeForwardForTime(5.0, 1.0);
        Actions.runBlocking(withPoseTelemetry(part2ParallelDriveAndIntake, drive));

        Pose2d afterPickup = drive.localizer.getPose();

        Action returnToShootPose = drive.actionBuilder(afterPickup)
                .splineToLinearHeading(shootPose, 0.0)
                .build();
        Actions.runBlocking(withPoseTelemetry(returnToShootPose, drive));

//        Action part2Shoot = new SequentialAction(
//                spinAndShoot,
//                intakeShootComboReverse1,
//                intakeShootComboBall1,
//                intakeShootComboReverse2,
//                intakeShootComboBall2,
//                intakeShootComboReverse3,
//                intakeShootComboBall3
//        );
//        Actions.runBlocking(withPoseTelemetry(part2Shoot, drive));


//// ===== PART 2 (optimized motion + intake parallel) =====
//// Key optimization: remove the in-place "turn(-138)" by entering the lane with a spline and finishing at pickupPose.
////
//// Since part1Move ends at stagePose deterministically, start part2 from stagePose (more repeatable than localizer pose).
//        Action part2DriveToPickup = drive.actionBuilder(stagePose)
//
//                // Enter the intake lane while rotating to intake heading.
//                // endTangent should match the direction you want to be traveling at pickup.
//                // If you are driving "upfield" toward +Y to reach y=45, tangent = +90° (PI/2).
//                .splineToLinearHeading(pickupPose, Math.PI / 2)
//
//                .build();
//
//// Intake runs while driving (deadline = driving action ends)
//        Action part2ParallelDriveAndIntake = new ParallelAction(
//                part2DriveToPickup,
//                intake.intakeForwardForTime(3.0, 1.0)
//        );
//
//// Run Part 4
//        Actions.runBlocking(withPoseTelemetry(part2ParallelDriveAndIntake, drive));

//        Pose2d stagePose = new Pose2d(45, 0, Math.toRadians(225));
//
//        double intakeTurnRad = Math.toRadians(-135);
//
//// If your Pose2d heading is a double in your RR build:
//        double intakeHeading = stagePose.heading.toDouble() + intakeTurnRad;
//
//// If your Pose2d heading is Rotation2d, use:
//// double intakeHeading = stagePose.heading.toDouble() + intakeTurnRad;
//
//        double dist = 48.0;
//
//// If the final heading is ~90°, this reduces to y + 48, but compute generically:
//        double targetX = stagePose.position.x + dist * Math.cos(intakeHeading);
//        double targetY = stagePose.position.y + dist * Math.sin(intakeHeading);
//
//        Pose2d pickupPose = new Pose2d(targetX, targetY, intakeHeading);
//
//// Drive + rotate in one continuous motion.
//// endTangent should match the direction of travel at the end; if you are arriving traveling along +Y, use PI/2.
//        Action part2DriveToPickup = drive.actionBuilder(stagePose)
//                .splineToLinearHeading(pickupPose, Math.PI / 2)
//                .build();
//
//        Action part2Parallel = new ParallelAction(
//                part2DriveToPickup,
//                intake.intakeForwardForTime(3.0, 1.0)
//        );
//
//        Actions.runBlocking(withPoseTelemetry(part2Parallel, drive));


    }

    public class ShootAndIntakeAction {

        private final Shooter shooter;
        private final Intake intake;
        private final double durationSeconds;
        private final double intakeDirection;

        public ShootAndIntakeAction(Shooter shooter, Intake intake, double durationSeconds, double intakeDirection) {
            this.shooter = shooter;
            this.intake = intake;
            this.durationSeconds = durationSeconds;
            this.intakeDirection = intakeDirection;
        }

        public Action buildShootIntake() {
            return new ParallelAction(
                    shooter.spinBangBangForTime(durationSeconds),
                    intake.intakeForwardForTime(durationSeconds, intakeDirection)
            );
        }
    }

    private Action withPoseTelemetry(Action inner, MecanumDrive drive) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // 1) Do telemetry
                Pose2d pose = drive.localizer.getPose();

                packet.put("X", pose.position.x);
                packet.put("Y", pose.position.y);
                packet.put("HeadingDeg", Math.toDegrees(pose.heading.toDouble()));

                telemetry.addData("X", pose.position.x);
                telemetry.addData("Y", pose.position.y);
                telemetry.addData("Heading (deg)", Math.toDegrees(pose.heading.toDouble()));
                telemetry.update();

                // 2) Delegate to the wrapped action
                return inner.run(packet);
            }
        };
    }
}

