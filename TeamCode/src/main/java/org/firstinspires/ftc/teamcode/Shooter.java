package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.util.Iterator;

public class Shooter {

    private final DcMotorEx shooterR;
    private final Servo trapdoor;

    // Trapdoor state (latched)
    private boolean trapdoorOpen = false;

    // --- TRAPDOOR POSITIONS (your robot) ---
    private static final double TRAPDOOR_CLOSED_POS = 1.0;
    private static final double TRAPDOOR_OPEN_POS   = 0.0;

    // --- ENCODER ---
    private static final double ENCODER_COUNTS_PER_REV = 28.0;

    // --- PIDF (tuned) ---
    private static final double shooterP = 10.0;
    private static final double shooterI = 0.0;
    private static final double shooterD = 0.0;
    private static final double shooterF = 13.22;

    // Voltage compensation
    private VoltageSensor batteryVoltageSensor;
    private static final double V_BASE = 12.86;          // voltage you tuned F at
    private static final double F_BASE_NEAR = shooterF;  // tuned F at V_BASE
    private static double F_BASE = F_BASE_NEAR;
    private static final double F_UPDATE_EPS = 0.05;
    private double lastFApplied = -1.0;

    // NOTE: treated as TARGET RPM (kept name)
    public double BANG_BANG_TARGET_VELOCITY;

    // Exposed for telemetry
    private double actualRPM = 0.0;

    // Shooter enable/disable
    private boolean enabled = true;
    private boolean wasEnabled = true;

    public Shooter(HardwareMap hardwareMap, double BANG_BANG_TARGET_VELOCITY) {
        shooterR = hardwareMap.get(DcMotorEx.class, "shooterR");
        trapdoor = hardwareMap.get(Servo.class, "trapdoor");

        Iterator<VoltageSensor> it = hardwareMap.voltageSensor.iterator();
        if (it.hasNext()) {
            batteryVoltageSensor = it.next();
        }

        shooterR.setDirection(DcMotor.Direction.REVERSE);
        shooterR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooterR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients pidf = new PIDFCoefficients(
                shooterP, shooterI, shooterD, shooterF, MotorControlAlgorithm.PIDF
        );
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

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;

        if (!enabled) {
            // Immediately stop and close trapdoor
            shooterR.setVelocity(0);
            actualRPM = 0.0;
            trapdoorOpen = false;
            trapdoor.setPosition(TRAPDOOR_CLOSED_POS);
        }
    }

    public boolean isEnabled() {
        return enabled;
    }

    private static double rpmToTicksPerSec(double rpm) {
        return (rpm / 60.0) * ENCODER_COUNTS_PER_REV;
    }

    private static double ticksPerSecToRpm(double ticksPerSec) {
        return (ticksPerSec / ENCODER_COUNTS_PER_REV) * 60.0;
    }

    /**
     * Continuous shooter update (Auton/TeleOp):
     * - Always commands velocity to target RPM (unless disabled).
     * - Trapdoor policy (NO RPM gating):
     *     - allowTrapdoor == false (moving)   => force CLOSED
     *     - allowTrapdoor == true  (shooting) => OPEN and stay open
     */
    public void update(boolean allowTrapdoor) {

        // If disabled, force everything off and return fast
        if (!enabled) {
            shooterR.setVelocity(0);
            actualRPM = 0.0;
            trapdoorOpen = false;
            trapdoor.setPosition(TRAPDOOR_CLOSED_POS);
            wasEnabled = false;
            return;
        }

        // If we just re-enabled, ensure we start closed
        if (!wasEnabled) {
            trapdoorOpen = false;
            trapdoor.setPosition(TRAPDOOR_CLOSED_POS);
            wasEnabled = true;
        }

        // Always command shooter velocity (continuous run)
        applyShooterPIDFWithVoltageComp(shooterR);
        shooterR.setVelocity(rpmToTicksPerSec(BANG_BANG_TARGET_VELOCITY));

        // Measure RPM for telemetry
        double actualTicksPerSec = shooterR.getVelocity();
        actualRPM = ticksPerSecToRpm(actualTicksPerSec);

        // Trapdoor policy (NO RPM gating)
        trapdoorOpen = allowTrapdoor;
        trapdoor.setPosition(trapdoorOpen ? TRAPDOOR_OPEN_POS : TRAPDOOR_CLOSED_POS);
    }

    private void applyShooterPIDFWithVoltageComp(DcMotorEx shooter) {
        if (batteryVoltageSensor == null) return;

        double vNow = batteryVoltageSensor.getVoltage();
        if (vNow < 1.0) vNow = V_BASE; // safety fallback

        // Scale feedforward as voltage drops: F increases
        double fScaled = F_BASE * (V_BASE / vNow);

        PIDFCoefficients pidf = new PIDFCoefficients(
                shooterP, shooterI, shooterD, fScaled, MotorControlAlgorithm.PIDF
        );

        // Avoid spamming the hub every loop
        if (lastFApplied < 0 || Math.abs(fScaled - lastFApplied) > F_UPDATE_EPS) {
            shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
            lastFApplied = fScaled;
        }
    }

    /** Explicit stop */
    public void stop() {
        shooterR.setVelocity(0);
        actualRPM = 0.0;
        trapdoorOpen = false;
        trapdoor.setPosition(TRAPDOOR_CLOSED_POS);
    }
}

