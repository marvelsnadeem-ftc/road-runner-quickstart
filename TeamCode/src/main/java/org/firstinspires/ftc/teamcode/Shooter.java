package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Shooter {
    private DcMotorEx shooterR;

    // NOTE: This is now treated as TARGET RPM (not ticks/sec).
    private static final double BANG_BANG_TARGET_VELOCITY = 2800; // RPM target you requested

    // Kept to minimize changes; no longer used in PIDF velocity mode
    //private static final double FLYWHEEL_FULL_POWER       = -0.58;

    // -----------------------------
    // REQUIRED for velocity PIDF
    // -----------------------------
    // You measured: 10 wheel revs -> 280 counts => 28 counts/rev
    private static final double ENCODER_COUNTS_PER_REV = 28.0;

    // You measured max getVelocity() at full power:
    private static final double MAX_TICKS_PER_SEC_MEASURED = 2400.0;

    // Feedforward: F = 32767 / maxTicksPerSec
    private static final double shooterF = 32767.0 / MAX_TICKS_PER_SEC_MEASURED; // 13.653

    // Start values (tune P upward for faster recovery; keep I=0 initially)
    private static final double shooterP = 7.0;
    private static final double shooterI = 3.0;
    private static final double shooterD = 1.5;

    public Shooter(HardwareMap hardwareMap) {
        shooterR = hardwareMap.get(DcMotorEx.class, "shooterR");

        shooterR.setDirection(DcMotor.Direction.REVERSE);
        shooterR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooterR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // REQUIRED: velocity PIDF uses RUN_USING_ENCODER
        shooterR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // REQUIRED: enable PIDF for velocity control
        shooterR.setVelocityPIDFCoefficients(shooterP, shooterI, shooterD, shooterF);
    }

    /**
     * Spin and hold using velocity PIDF for a fixed duration.
     * (Function name kept the same.)
     */
    public class SpinBangBangForTime implements Action {
        private final double durationSeconds;
        private boolean initialized = false;
        private final ElapsedTime timer = new ElapsedTime();

        public SpinBangBangForTime(double durationSeconds) {
            this.durationSeconds = durationSeconds;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                timer.reset();
                initialized = true;
            }

            // Convert target RPM -> target ticks/sec for setVelocity()
            double targetTicksPerSec =
                    (BANG_BANG_TARGET_VELOCITY / 60.0) * ENCODER_COUNTS_PER_REV;

            // Command velocity (ticks/sec)
            shooterR.setVelocity(targetTicksPerSec);

            // Read velocity (ticks/sec) and convert back to RPM for telemetry
            double actualTicksPerSec = shooterR.getVelocity();
            double actualRPM = (actualTicksPerSec / ENCODER_COUNTS_PER_REV) * 60.0;

            packet.put("time", timer.seconds());
            packet.put("targetRPM", BANG_BANG_TARGET_VELOCITY);
            packet.put("targetTicksPerSec", targetTicksPerSec);
            packet.put("actualTicksPerSec", actualTicksPerSec);
            packet.put("actualRPM", actualRPM);

            // (Optional but useful) show PIDF being used
            packet.put("P", shooterP);
            packet.put("I", shooterI);
            packet.put("D", shooterD);
            packet.put("F", shooterF);

            if (timer.seconds() < durationSeconds) {
                return true;
            } else {
                shooterR.setVelocity(0);
                return false;
            }
        }
    }

    public Action spinBangBangForTime(double durationSeconds) {
        return new SpinBangBangForTime(durationSeconds);
    }

    /**
     * Stop the shooter immediately.
     */
    public class StopShooter implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            shooterR.setVelocity(0);

            double actualTicksPerSec = shooterR.getVelocity();
            double actualRPM = (actualTicksPerSec / ENCODER_COUNTS_PER_REV) * 60.0;

            packet.put("actualTicksPerSec", actualTicksPerSec);
            packet.put("actualRPM", actualRPM);
            return false;
        }
    }

    public Action stopShooter() {
        return new StopShooter();
    }
}



//package org.firstinspires.ftc.teamcode;
//
//import androidx.annotation.NonNull;
//
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Action;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//public class Shooter {
//    private DcMotorEx shooterR;
//
//    // From your TeleOp
//    private static final double BANG_BANG_TARGET_VELOCITY = 1500.0;
//    private static final double FLYWHEEL_FULL_POWER       = -0.58;
//
//    public Shooter(HardwareMap hardwareMap) {
//        shooterR = hardwareMap.get(DcMotorEx.class, "shooterR");
//
//        shooterR.setDirection(DcMotor.Direction.FORWARD);
//        shooterR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//
//        shooterR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        shooterR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//    }
//
//    /**
//     * Spin and hold using the same bang-bang logic as TeleOp for a fixed duration.
//     */
//    public class SpinBangBangForTime implements Action {
//        private final double durationSeconds;
//        private boolean initialized = false;
//        private final ElapsedTime timer = new ElapsedTime();
//
//        public SpinBangBangForTime(double durationSeconds) {
//            this.durationSeconds = durationSeconds;
//        }
//
//        @Override
//        public boolean run(@NonNull TelemetryPacket packet) {
//            if (!initialized) {
//                timer.reset();
//                initialized = true;
//            }
//
//            double currentVelocity = -shooterR.getVelocity(); // same sign as in TeleOp
//            packet.put("shooterVel", currentVelocity);
//            packet.put("targetVel", BANG_BANG_TARGET_VELOCITY);
//            packet.put("time", timer.seconds());
//
//            if (currentVelocity < BANG_BANG_TARGET_VELOCITY) {
//                shooterR.setPower(FLYWHEEL_FULL_POWER);
//            } else {
//                shooterR.setPower(0);
//            }
//
//            if (timer.seconds() < durationSeconds) {
//                return true;
//            } else {
//                shooterR.setPower(0);
//                return false;
//            }
//        }
//    }
//
//    public Action spinBangBangForTime(double durationSeconds) {
//        return new SpinBangBangForTime(durationSeconds);
//    }
//
//    /**
//     * Stop the shooter immediately.
//     */
//    public class StopShooter implements Action {
//        @Override
//        public boolean run(@NonNull TelemetryPacket packet) {
//            shooterR.setPower(0);
//            packet.put("shooterVel", -shooterR.getVelocity());
//            return false;
//        }
//    }
//
//    public Action stopShooter() {
//        return new StopShooter();
//    }
//}
