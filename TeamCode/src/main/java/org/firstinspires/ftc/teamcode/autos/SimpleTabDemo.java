package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "Simple_TrajectoryActionBuilder_Demo", group = "RoadRunner")
public class SimpleTabDemo extends LinearOpMode {

    // ==== CHANGE THESE TO WHATEVER YOU WANT ====
    private static final double FIRST_FORWARD_IN   = 24.0;  // inches
    private static final double FIRST_TURN_DEG     = 90.0;  // left turn
    private static final double SECOND_FORWARD_IN  = 24.0;  // inches
    private static final double SECOND_TURN_DEG    = 90.0;  // left turn
    private static final double THIRD_FORWARD_IN   = 24.0;  // inches
    // ===========================================

    @Override
    public void runOpMode() throws InterruptedException {

        // Start pose: origin (0, 0), heading 0 rad.
        // Units are inches by convention.
        Pose2d startPose = new Pose2d(0.0, 0.0, 0.0);

        // Your RR 1.0 MecanumDrive class from quickstart
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        // Build a single trajectory action with:
        // forward -> left turn -> forward -> left turn -> forward
        TrajectoryActionBuilder tab = drive.actionBuilder(startPose)
                // 1) Move forward FIRST_FORWARD_IN (along +X from 0 to FIRST_FORWARD_IN)
                .lineToX(FIRST_FORWARD_IN)

                // 2) Turn left FIRST_TURN_DEG
                .turn(Math.toRadians(FIRST_TURN_DEG))

                // After this, heading is 90° (π/2). We want to drive "forward" from here,
                // so set the tangent along +Y.
                .setTangent(Math.toRadians(90.0))

                // 3) Move forward SECOND_FORWARD_IN from y = 0 -> SECOND_FORWARD_IN
                .lineToY(SECOND_FORWARD_IN)

                // 4) Turn left SECOND_TURN_DEG
                .turn(Math.toRadians(SECOND_TURN_DEG))
                // Now heading is 180°. "Forward" is along -X.
                .setTangent(Math.toRadians(180.0))

                // We started at x = FIRST_FORWARD_IN, so to move THIRD_FORWARD_IN forward
                // in that -X direction, we go to x = FIRST_FORWARD_IN - THIRD_FORWARD_IN.
                .lineToX(FIRST_FORWARD_IN - THIRD_FORWARD_IN);

        // Convert builder into an Action
        Action drivePath = tab.build();

        telemetry.addLine("Ready - press play");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Run the whole path once, then stop
        Actions.runBlocking(drivePath);

        // After this, Road Runner stops commanding the drive (robot will brake/coast
        // based on your motor zeroPowerBehavior settings).
    }
}

