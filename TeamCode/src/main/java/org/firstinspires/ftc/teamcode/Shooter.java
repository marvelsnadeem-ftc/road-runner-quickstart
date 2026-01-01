package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

public class Shooter {
    private final DcMotorEx shooterR;
    private final Servo trapdoor;

    // Trapdoor state
    private boolean trapdoorOpen = false;

    // --- TRAPDOOR POSITIONS (your robot) ---
    private static final double TRAPDOOR_CLOSED_POS = 1.0;
    private static final double TRAPDOOR_OPEN_POS   = 0.0;

    // --- HYSTERESIS / GATING THRESHOLDS ---
    // Open when within +/-100 RPM of target
    private static final double OPEN_TOL_RPM   = 100.0;

    // Close immediately if droop is large (after shot)
    private static final double CLOSE_LOW_RPM  = 320.0;  // closes if err <= -320

    // Close immediately if overspeed exceeds +100
    private static final double CLOSE_HIGH_RPM = 100.0;  // closes if err >= +100

    // --- ENCODER + MEASURED LIMITS ---
    private static final double ENCODER_COUNTS_PER_REV = 28.0;

    // --- PIDF (tuned) ---
    private static final double shooterP = 8.0;
    private static final double shooterI = 0.0;
    private static final double shooterD = 0.0;
    private static final double shooterF = 14.53;// (32767.0 / MAX_TICKS_PER_SEC_MEASURED) * 1.064 = 14.53 //(32767.0 / MAX_TICKS_PER_SEC_MEASURED) * 1.106 = 15.10 // 13.653 // keep your tuned value

    // NOTE: treated as TARGET RPM (kept name)
    public double BANG_BANG_TARGET_VELOCITY;

    // Exposed for telemetry
    private double actualRPM = 0.0;

    public Shooter(HardwareMap hardwareMap, double BANG_BANG_TARGET_VELOCITY) {
        shooterR = hardwareMap.get(DcMotorEx.class, "shooterR");
        trapdoor = hardwareMap.get(Servo.class, "trapdoor");

        shooterR.setDirection(DcMotor.Direction.REVERSE);
        shooterR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooterR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients pidf = new PIDFCoefficients(shooterP, shooterI, shooterD, shooterF, MotorControlAlgorithm.PIDF);
        shooterR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        this.BANG_BANG_TARGET_VELOCITY = BANG_BANG_TARGET_VELOCITY;

        // Safe defaults
        trapdoorOpen = false;
        trapdoor.setPosition(TRAPDOOR_CLOSED_POS);
        shooterR.setVelocity(0);
    }

    public double getActualRPM() {
        return actualRPM;
    }

    public double getBangBangTargetVelocity() {
        return BANG_BANG_TARGET_VELOCITY;
    }

    private static double rpmToTicksPerSec(double rpm) {
        return (rpm / 60.0) * ENCODER_COUNTS_PER_REV;
    }

    private static double ticksPerSecToRpm(double ticksPerSec) {
        return (ticksPerSec / ENCODER_COUNTS_PER_REV) * 60.0;
    }

    /**
     * Continuous shooter update for autonomous:
     * - Always commands velocity to target RPM.
     * - Trapdoor is forced CLOSED while moving.
     * - Trapdoor only opens in shootingPose when RPM is within tolerance,
     *   and closes on overspeed (+100) or droop (target - 320).
     */
    public void update(boolean allowTrapdoor) {
        // Always command velocity (continuous run)
        double targetTicksPerSec = rpmToTicksPerSec(BANG_BANG_TARGET_VELOCITY);
        shooterR.setVelocity(targetTicksPerSec);

        // Measure RPM
        double actualTicksPerSec = shooterR.getVelocity();
        actualRPM = ticksPerSecToRpm(actualTicksPerSec);

        // If robot is moving, force trapdoor closed
        if (!allowTrapdoor) {
            trapdoorOpen = false;
            trapdoor.setPosition(TRAPDOOR_CLOSED_POS);
            return;
        }

        // Trapdoor logic only in shootingPose
        double err = actualRPM - BANG_BANG_TARGET_VELOCITY;

        // Close immediately if overspeed (> +100) OR big droop (< -320)
        if (err >= CLOSE_HIGH_RPM || err <= -CLOSE_LOW_RPM) {
            trapdoorOpen = false;
        } else {
            // Otherwise, open only when we are in the "vicinity" window
            if (Math.abs(err) <= OPEN_TOL_RPM) {
                trapdoorOpen = true;
            }
        }

        trapdoor.setPosition(trapdoorOpen ? TRAPDOOR_OPEN_POS : TRAPDOOR_CLOSED_POS);
    }

    /** Optional explicit stop (not required for your new auton flow) */
    public void stop() {
        shooterR.setVelocity(0);
        trapdoorOpen = false;
        trapdoor.setPosition(TRAPDOOR_CLOSED_POS);
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
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
//import com.qualcomm.robotcore.hardware.PIDFCoefficients;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//public class Shooter {
//    private final DcMotorEx shooterR;
//    private final Servo trapdoor;
//
//    // Trapdoor state (required for hysteresis)
//    private boolean trapdoorOpen = false;
//
//    // --- TRAPDOOR POSITIONS (your robot) ---
//    private static final double TRAPDOOR_CLOSED_POS = 1.0;
//    private static final double TRAPDOOR_OPEN_POS   = 0.0;
//
//    // --- HYSTERESIS / GATING THRESHOLDS ---
//    // Open when within +/-100 RPM of target
//    private static final double OPEN_TOL_RPM = 100.0;
//
//    // Close immediately if droop is large (after shot)
//    private static final double CLOSE_LOW_RPM = 320.0;     // closes if error <= -320
//
//    // Close immediately if overspeed exceeds +100 (your requirement)
//    private static final double CLOSE_HIGH_RPM = 100.0;    // closes if error >= +100
//
//
//    // --- SPEED GATING ---
//    private static final double RPM_TOLERANCE = 100.0;     // open if within +/- 100 rpm
//    private static final double RPM_SETTLE_SEC = 0.15;    // must be in-band for 150ms before opening
//
//    // --- ENCODER + MEASURED LIMITS ---
//    private static final double ENCODER_COUNTS_PER_REV = 28.0;
//    private static final double MAX_TICKS_PER_SEC_MEASURED = 2400.0;
//
//    // --- PIDF (tuned) ---
//    private static final double shooterP = 8.0;
//    private static final double shooterI = 0.0;
//    private static final double shooterD = 0.0;
//    private static final double shooterF = 14.53; // (32767.0 / MAX_TICKS_PER_SEC_MEASURED) * 1.064 = 14.53 //(32767.0 / MAX_TICKS_PER_SEC_MEASURED) * 1.106 = 15.10 // 13.653
//
//    // NOTE: treated as TARGET RPM (kept name to minimize disruption)
//    public double BANG_BANG_TARGET_VELOCITY;
//
//    // Exposed for telemetry (kept behavior)
//    private double actualRPM = 0.0;
//
//    public Shooter(HardwareMap hardwareMap, double BANG_BANG_TARGET_VELOCITY) {
//        shooterR = hardwareMap.get(DcMotorEx.class, "shooterR");
//        trapdoor = hardwareMap.get(Servo.class, "trapdoor");
//
//        shooterR.setDirection(DcMotor.Direction.REVERSE);
//        shooterR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//
//        shooterR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        shooterR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        PIDFCoefficients pidf = new PIDFCoefficients(shooterP, shooterI, shooterD, shooterF, MotorControlAlgorithm.PIDF);
//        shooterR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
//
//        this.BANG_BANG_TARGET_VELOCITY = BANG_BANG_TARGET_VELOCITY;
//
//        // Safe default
//        trapdoor.setPosition(TRAPDOOR_CLOSED_POS);
//        shooterR.setVelocity(0);
//    }
//
//    public double getActualRPM() {
//        return actualRPM;
//    }
//
//    public double getBangBangTargetVelocity() {
//        return BANG_BANG_TARGET_VELOCITY;
//    }
//
//    private static double rpmToTicksPerSec(double rpm) {
//        return (rpm / 60.0) * ENCODER_COUNTS_PER_REV;
//    }
//
//    private static double ticksPerSecToRpm(double ticksPerSec) {
//        return (ticksPerSec / ENCODER_COUNTS_PER_REV) * 60.0;
//    }
//
//    /**
//     * Spin and hold using velocity PIDF for a fixed duration.
//     * (Function name kept the same.)
//     */
//    public class SpinBangBangForTime implements Action {
//        private final double durationSeconds;
//        private boolean initialized = false;
//
//        private final ElapsedTime timer = new ElapsedTime();
//        private final ElapsedTime inBandTimer = new ElapsedTime();
//        private boolean inBandTiming = false;
//
//        public SpinBangBangForTime(double durationSeconds) {
//            this.durationSeconds = durationSeconds;
//        }
//
//        @Override
//        public boolean run(@NonNull TelemetryPacket packet) {
//
//            if (!initialized) {
//                timer.reset();
//                trapdoorOpen = false;
//                trapdoor.setPosition(TRAPDOOR_CLOSED_POS);
//                initialized = true;
//            }
//
//            // Command velocity
//            double targetTicksPerSec = rpmToTicksPerSec(BANG_BANG_TARGET_VELOCITY);
//            shooterR.setVelocity(targetTicksPerSec);
//
//            // Measure
//            double actualTicksPerSec = shooterR.getVelocity();
//            actualRPM = ticksPerSecToRpm(actualTicksPerSec);
//
//            // ----------------------------
//            // NEW TRAPDOOR HYSTERESIS LOGIC
//            // ----------------------------
//            double err = actualRPM - BANG_BANG_TARGET_VELOCITY;
//
//            // Close immediately if overspeed (> +100) OR big droop (< -320)
//            if (err >= CLOSE_HIGH_RPM || err <= -CLOSE_LOW_RPM) {
//                trapdoorOpen = false;
//            } else {
//                // Otherwise, open only when we are in the "vicinity" window
//                if (Math.abs(err) <= OPEN_TOL_RPM) {
//                    trapdoorOpen = true;
//                }
//            }
//
//            // Command servo from the state
//            trapdoor.setPosition(trapdoorOpen ? TRAPDOOR_OPEN_POS : TRAPDOOR_CLOSED_POS);
//
//
//            // Telemetry
//            packet.put("time", timer.seconds());
//            packet.put("targetRPM", BANG_BANG_TARGET_VELOCITY);
//            packet.put("actualRPM", actualRPM);
//            packet.put("targetTicksPerSec", targetTicksPerSec);
//            packet.put("actualTicksPerSec", actualTicksPerSec);
//            packet.put("rpmTol", RPM_TOLERANCE);
//            packet.put("settleSec", RPM_SETTLE_SEC);
//            packet.put("P", shooterP);
//            packet.put("I", shooterI);
//            packet.put("D", shooterD);
//            packet.put("F", shooterF);
//
//            if (timer.seconds() < durationSeconds) {
//                return true;
//            } else {
//                shooterR.setVelocity(0);
//                trapdoor.setPosition(TRAPDOOR_CLOSED_POS);
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
//     * Stop the shooter immediately (also closes trapdoor).
//     */
//    public class StopShooter implements Action {
//        @Override
//        public boolean run(@NonNull TelemetryPacket packet) {
//            shooterR.setVelocity(0);
//            trapdoor.setPosition(TRAPDOOR_CLOSED_POS);
//
//            double actualTicksPerSec = shooterR.getVelocity();
//            double rpm = ticksPerSecToRpm(actualTicksPerSec);
//
//            packet.put("actualTicksPerSec", actualTicksPerSec);
//            packet.put("actualRPM", rpm);
//            return false;
//        }
//    }
//
//    public Action stopShooter() {
//        return new StopShooter();
//    }
//}
//
//
//
//
//
//
//
////package org.firstinspires.ftc.teamcode;
////
////import androidx.annotation.NonNull;
////
////import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
////import com.acmerobotics.roadrunner.Action;
////import com.qualcomm.robotcore.hardware.DcMotor;
////import com.qualcomm.robotcore.hardware.DcMotorEx;
////import com.qualcomm.robotcore.hardware.HardwareMap;
////import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
////import com.qualcomm.robotcore.hardware.PIDFCoefficients;
////import com.qualcomm.robotcore.hardware.Servo;
////import com.qualcomm.robotcore.util.ElapsedTime;
////
////public class Shooter {
////    private DcMotorEx shooterR;
////    // Servo
////    private Servo trapdoor = null;
////    private boolean trapdoorOpen = false;
////    // --- TRAPDOOR POSITIONS (adjust to your robot) ---
////    private static final double TRAPDOOR_CLOSED_POS = 1.0;
////    private static final double TRAPDOOR_OPEN_POS   = 0.0;
////
////    // NOTE: This is now treated as TARGET RPM (not ticks/sec).
////    public  double BANG_BANG_TARGET_VELOCITY; // RPM target you requested
////    private double actualRPM;
////    // Kept to minimize changes; no longer used in PIDF velocity mode
////    //private static final double FLYWHEEL_FULL_POWER       = -0.58;
////
////    // -----------------------------
////    // REQUIRED for velocity PIDF
////    // -----------------------------
////    // You measured: 10 wheel revs -> 280 counts => 28 counts/rev
////    private static final double ENCODER_COUNTS_PER_REV = 28.0;
////
////    // You measured max getVelocity() at full power:
////    private static final double MAX_TICKS_PER_SEC_MEASURED = 2400.0;
////
////    // Feedforward: F = 32767 / maxTicksPerSec
////    private static final double shooterF = (32767.0 / MAX_TICKS_PER_SEC_MEASURED) * 1.106; //15.10 // 13.653
////
////    // Start values (tune P upward for faster recovery; keep I=0 initially)
////    private static final double shooterP = 7.0;
////    private static final double shooterI = 0.0;
////    private static final double shooterD = 0.0;
////
////    public Shooter(HardwareMap hardwareMap, double BANG_BANG_TARGET_VELOCITY) {
////        shooterR = hardwareMap.get(DcMotorEx.class, "shooterR");
////        // Servo
////        trapdoor = hardwareMap.get(Servo.class, "trapdoor");
////
////        shooterR.setDirection(DcMotor.Direction.REVERSE);
////        shooterR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
////
////        shooterR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////
////        // REQUIRED: velocity PIDF uses RUN_USING_ENCODER
////        shooterR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
////
////        // Use explicit PIDF set for RUN_USING_ENCODER
////        PIDFCoefficients pidf = new PIDFCoefficients(shooterP, shooterI, shooterD, shooterF, MotorControlAlgorithm.PIDF);
////        shooterR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
////
////        this.BANG_BANG_TARGET_VELOCITY = BANG_BANG_TARGET_VELOCITY;
////
////    }
////    public double getActualRPM()
////    {
////        return actualRPM;
////    }
////    public double getBangBangTargetVelocity()
////    {
////        return BANG_BANG_TARGET_VELOCITY;
////    }
////    /**
////     * Spin and hold using velocity PIDF for a fixed duration.
////     * (Function name kept the same.)
////     */
////    public class SpinBangBangForTime implements Action {
////        private final double durationSeconds;
////        private boolean initialized = false;
////        private final ElapsedTime timer = new ElapsedTime();
////
////        public SpinBangBangForTime(double durationSeconds) {
////            this.durationSeconds = durationSeconds;
////        }
////
////        @Override
////        public boolean run(@NonNull TelemetryPacket packet) {
////            if (!initialized) {
////                timer.reset();
////                initialized = true;
////            }
////            trapdoor.setPosition(TRAPDOOR_CLOSED_POS);
////            // Convert target RPM -> target ticks/sec for setVelocity()
////            double targetTicksPerSec =
////                    (BANG_BANG_TARGET_VELOCITY / 60.0) * ENCODER_COUNTS_PER_REV;
////
////            // Command velocity (ticks/sec)
////            shooterR.setVelocity(targetTicksPerSec);
////
////            // Read velocity (ticks/sec) and convert back to RPM for telemetry
////            double actualTicksPerSec = shooterR.getVelocity();
////            actualRPM = (actualTicksPerSec / ENCODER_COUNTS_PER_REV) * 60.0;
////
////            trapdoor.setPosition(
////                    Math.abs(actualRPM - BANG_BANG_TARGET_VELOCITY) <= 50.0
////                            ? TRAPDOOR_OPEN_POS
////                            : TRAPDOOR_CLOSED_POS
////            );
////
////            packet.put("time", timer.seconds());
////            packet.put("targetRPM", BANG_BANG_TARGET_VELOCITY);
////            packet.put("targetTicksPerSec", targetTicksPerSec);
////            packet.put("actualTicksPerSec", actualTicksPerSec);
////            packet.put("actualRPM", actualRPM);
////
////            // (Optional but useful) show PIDF being used
////            packet.put("P", shooterP);
////            packet.put("I", shooterI);
////            packet.put("D", shooterD);
////            packet.put("F", shooterF);
////
////            if (timer.seconds() < durationSeconds) {
////                return true;
////            } else {
////                shooterR.setVelocity(0);
////                return false;
////            }
////        }
////    }
////
////    public Action spinBangBangForTime(double durationSeconds) {
////        return new SpinBangBangForTime(durationSeconds);
////    }
////
////    /**
////     * Stop the shooter immediately.
////     */
////    public class StopShooter implements Action {
////        @Override
////        public boolean run(@NonNull TelemetryPacket packet) {
////            shooterR.setVelocity(0);
////
////            double actualTicksPerSec = shooterR.getVelocity();
////            double actualRPM = (actualTicksPerSec / ENCODER_COUNTS_PER_REV) * 60.0;
////
////            packet.put("actualTicksPerSec", actualTicksPerSec);
////            packet.put("actualRPM", actualRPM);
////            return false;
////        }
////    }
////
////    public Action stopShooter() {
////        return new StopShooter();
////    }
////}
//
//
