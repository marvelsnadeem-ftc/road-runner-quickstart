package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Intake {
    private DcMotor intake;
    private static final double INTAKE_POWER = 1.0;

    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotor.Direction.REVERSE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setPower(double power) {
        intake.setPower(power);
    }
    /**
     * Run intake forward for a given number of seconds
     * (used here only to feed preloaded artifacts into the shooter).
     */
    public class IntakeForwardForTime implements Action {
        private final double durationSeconds;
        private boolean initialized = false;
        private double intakeDirection;
        private final ElapsedTime timer = new ElapsedTime();

        public IntakeForwardForTime(double durationSeconds, double intakeDirection) {
            this.durationSeconds = durationSeconds;
            this.intakeDirection = intakeDirection;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                intake.setPower(intakeDirection * INTAKE_POWER);
                timer.reset();
                initialized = true;
            }

            double t = timer.seconds();
            packet.put("intakeTime", t);

            if (t < durationSeconds) {
                return true;   // still feeding
            } else {
                intake.setPower(0);
                return false;  // done
            }
        }
    }

    public Action intakeForwardForTime(double durationSeconds, double intakeDirection) {
        return new IntakeForwardForTime(durationSeconds, intakeDirection);
    }

    /**
     * Immediately stop the intake.
     */
    public class IntakeStop implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intake.setPower(0);
            return false;
        }
    }

    public Action stopIntake() {
        return new IntakeStop();
    }
}
