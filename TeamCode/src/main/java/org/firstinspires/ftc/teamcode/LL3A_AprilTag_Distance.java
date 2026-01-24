package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.List;

@TeleOp(name = "LL3A AprilTag Distance (Lens)", group = "Test")
public class LL3A_AprilTag_Distance extends LinearOpMode {

    private Limelight3A limelight;

    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Use your AprilTag pipeline index here
        limelight.pipelineSwitch(0);

        // Poll faster if you want snappier updates
        limelight.setPollRateHz(100);

        telemetry.addLine("Ready (AprilTag pipeline + Full 3D should be enabled).");
        telemetry.update();

        waitForStart();
        limelight.start();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            if (result == null || !result.isValid()) {
                telemetry.addLine("No valid Limelight result");
                telemetry.update();
                continue;
            }

            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            if (fiducials == null || fiducials.isEmpty()) {
                telemetry.addLine("No AprilTags detected");
                telemetry.update();
                continue;
            }

            // Pick the closest tag by 3D lens distance
            LLResultTypes.FiducialResult best = null;
            double bestDistM = Double.POSITIVE_INFINITY;

            for (LLResultTypes.FiducialResult f : fiducials) {
                Pose3D tagPoseCam = f.getTargetPoseCameraSpace(); // tag pose relative to camera (lens)
                if (tagPoseCam == null) continue; // usually means Full 3D not enabled or no 3D solution

                Position p = tagPoseCam.getPosition();

                // Convert to meters robustly
                double xM = DistanceUnit.METER.fromUnit(p.unit, p.x);
                double yM = DistanceUnit.METER.fromUnit(p.unit, p.y);
                double zM = DistanceUnit.METER.fromUnit(p.unit, p.z);

                double distM = Math.sqrt(xM * xM + yM * yM + zM * zM);

                if (distM < bestDistM) {
                    bestDistM = distM;
                    best = f;
                }
            }

            if (best == null) {
                telemetry.addLine("Tags seen, but no 3D pose available (check Full 3D).");
                telemetry.update();
                continue;
            }

            double distanceCm = bestDistM * 100.0;
            double distanceIn = distanceCm / 2.54;

            telemetry.addData("Best Tag ID", best.getFiducialId());
            telemetry.addData("Lens Distance", "%.1f cm (%.2f in)", distanceCm, distanceIn);

            // Optional: also show aiming offsets from this tag detection
            telemetry.addData("tx (deg)", "%.2f", best.getTargetXDegrees());
            telemetry.addData("ty (deg)", "%.2f", best.getTargetYDegrees());

            telemetry.update();
        }
    }
}
