package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Shockwave Auton Red", group = "Auto")
public class ShockwaveRoadrunnerAutonRed extends LinearOpMode {
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
        //Pose2d initialPose = FieldCoordinates.BLUE_START_C1;
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

        // --- First part of trajectory ---
//        Action part1 = drive.actionBuilder(new Pose2d(0, 0, 0))
//                .lineToX(54)
//                .turn(Math.toRadians(-135))
//                .build();

// --- Last part of trajectory ---
//        Action part3 = drive.actionBuilder(new Pose2d(48, 0, Math.toRadians(-135)))
//                .turn(Math.toRadians(225))
//                .build();

// --- Run all of them in order ---
//        Actions.runBlocking(
//                new SequentialAction(
//                        part1,
//                        parallelDriveAndIntake,
//                        part3
//                )
//        );

        Action part1Base = drive.actionBuilder(new Pose2d(0, 0, 0))
                .lineToX(50)
                .turn(Math.toRadians(-212))
                .stopAndAdd(spinAndShoot)
                .stopAndAdd(intakeShootComboReverse1)
                .stopAndAdd(intakeShootComboBall1)
                .stopAndAdd(intakeShootComboReverse2)
                .stopAndAdd(intakeShootComboBall2)
                .stopAndAdd(intakeShootComboReverse3)
                .stopAndAdd(intakeShootComboBall3)
                .lineToX(45)
                //.turn(Math.toRadians(113))
                .build();

        // run part1
        Action part1 = withPoseTelemetry(part1Base, drive);
        Actions.runBlocking(part1);


        // --- Parallel segment: drive to X=48 and run intake 5 seconds ---
//        Action pickFrom3rdTile = drive.actionBuilder(new Pose2d(54, 0, Math.toRadians(90)))
//                .lineToY(54)
//                .build();

/*
//        Pose2d part2Pose = drive.localizer.getPose();
//        Action pickFrom3rdTile = drive.actionBuilder(part2Pose)
//                .lineToY(45)
//                .build();
//
//        Action parallelDriveAndIntakeBase =
//                new ParallelAction(
//                        pickFrom3rdTile,                       // deadline = driving
//                        intake.intakeForwardForTime(3.0, 1.0) // runs while driving
//                );
//        // run part2
//        Action parallelDriveAndIntake = withPoseTelemetry(parallelDriveAndIntakeBase, drive);
//        Actions.runBlocking(parallelDriveAndIntake);
//
//
//        Pose2d part3Pose = drive.localizer.getPose();
//        Action part3Base = drive.actionBuilder(part3Pose)
//                .lineToY(-5)
//                .turn(Math.toRadians(130))
//                .stopAndAdd(spinAndShootAfterPickup)
//                .stopAndAdd(intakeShootComboReverse4)
//                .stopAndAdd(intakeShootComboBall4)
//                .stopAndAdd(intakeShootComboReverse5)
//                .stopAndAdd(intakeShootComboBall5)
//                .stopAndAdd(intakeShootComboReverse6)
//                .stopAndAdd(intakeShootComboBall6)
//                .build();

        //                .turn(Math.toRadians(-135))

//         .stopAndAdd(intakeShootComboReverse4)
//                .stopAndAdd(intakeShootComboBall4)
//                .stopAndAdd(intakeShootComboReverse5)
//                .stopAndAdd(intakeShootComboBall5)
//                .stopAndAdd(intakeShootComboReverse6)
//                .stopAndAdd(intakeShootComboBall6)

        // run part2
        //Action part3 = withPoseTelemetry(part3Base, drive);
        Actions.runBlocking(part3Base);*/

//        Actions.runBlocking(
//                new SequentialAction(
//                        part1,
//                        parallelDriveAndIntake,
//                        part3,
//                        poseTelemetryAction
//                        ));


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

