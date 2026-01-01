package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.autos.FieldCoordinates;

/**
 * Helper for moving in TILE units, applying sign flips so the
 * commands match your robot's odometry frame, without modifying FieldCoordinates.
 *
 * BASE ROBOT CONVENTION (before flipping):
 *   - +X = forward (intake direction)
 *   - +Y = left
 *
 * You can flip X_SIGN and Y_SIGN at runtime depending on how the robot
 * is actually oriented on the field (intake upfield vs flywheel upfield, etc.).
 */
public final class TileMoveHelper {

    private TileMoveHelper() {}

    // Use the same tile size as the field map.
    public static final double TILE = FieldCoordinates.TILE;

    /**
     * SIGN CONFIGURATION (you can override these in your OpMode):
     *
     * We interpret tile movements in a FIELD semantic frame as:
     *   +rowsUp    = "upfield" (toward the goals)
     *   +colsLeft  = "left"   (toward column A)
     *
     * Then we convert them into ROBOT/ODOM frame deltas using these signs.
     *
     * BASE ASSUMPTION (intake-forward, left-positive):
     *   - "upfield" is aligned with robot +X (forward)  => X_SIGN = +1
     *   - "left"    is aligned with robot +Y (left)     => Y_SIGN = +1
     *
     * If your real robot behaves differently, you can flip:
     *   - X_SIGN = -1.0   // upfield is robot -X
     *   - Y_SIGN = -1.0   // left is robot -Y
     *
     * Example (flywheel facing upfield, intake backward):
     *   TileMoveHelper.X_SIGN = -1.0; // so "rowsUp" drives backwards
     */
    public static double X_SIGN = +1.0; // +rowsUp => +X by default
    public static double Y_SIGN = +1.0; // +left   => +Y by default

    /**
     * Build an Action that moves N rows upfield from the current pose.
     *
     * rowsUp > 0 => go "upfield" that many tile rows in FIELD terms.
     */
    public static Action moveRows(MecanumDrive drive, int rowsUp) {
        Pose2d p = drive.localizer.getPose();

        // In FIELD semantics, "upfield" is +rowsUp tiles.
        double deltaFieldForward = rowsUp * TILE;

        // Map field "forward" to robot X using sign.
        double deltaRobotX = X_SIGN * deltaFieldForward;

        double targetX = p.position.x + deltaRobotX;

        return drive.actionBuilder(p)
                .lineToX(targetX)
                .build();
    }

    /**
     * Build an Action that moves N columns left from the current pose.
     *
     * colsLeft > 0 => go "left" that many tile columns in FIELD terms.
     */
    public static Action moveColsLeft(MecanumDrive drive, int colsLeft) {
        Pose2d p = drive.localizer.getPose();

        // In FIELD semantics, "left" is +colsLeft tiles.
        double deltaFieldLeft = colsLeft * TILE;

        // Map field "left" to robot Y using sign.
        double deltaRobotY = Y_SIGN * deltaFieldLeft;

        double targetY = p.position.y + deltaRobotY;

        return drive.actionBuilder(p)
                .lineToY(targetY)
                .build();
    }

    /**
     * Move rows and columns together to a tile offset corner.
     * rowsUp > 0 => upfield
     * colsLeft > 0 => left
     *
     * Uses strafeTo(Vector2d) which exists in Road Runner v1.0.
     */
    public static Action moveRowsAndCols(MecanumDrive drive, int rowsUp, int colsLeft) {
        Pose2d p = drive.localizer.getPose();

        double deltaFieldForward = rowsUp * TILE;   // upfield in field semantics
        double deltaFieldLeft    = colsLeft * TILE; // left in field semantics

        double deltaRobotX = X_SIGN * deltaFieldForward;
        double deltaRobotY = Y_SIGN * deltaFieldLeft;

        double targetX = p.position.x + deltaRobotX;
        double targetY = p.position.y + deltaRobotY;

        return drive.actionBuilder(p)
                .strafeTo(new Vector2d(targetX, targetY))
                .build();
    }
}
