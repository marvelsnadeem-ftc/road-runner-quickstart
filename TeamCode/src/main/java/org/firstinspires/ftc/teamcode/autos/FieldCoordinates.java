package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.Pose2d;

/**
 * Pure field-centric tile coordinates for a 6x6 DECODE field.
 *
 * CONVENTION (agnostic of which side is intake/flywheel):
 *  - Origin (0,0) at the CENTER of the 6x6 tile area.
 *  - +X points UPFIELD (from row 1 -> row 6).
 *  - +Y points RIGHT (from column A -> column F).
 *  - Each tile is 24" x 24".
 *
 * Tile center coordinates (inches):
 *  - Rows (1..6) -> X = -60, -36, -12, +12, +36, +60
 *  - Cols  (A..F) -> Y = -60, -36, -12, +12, +36, +60
 *
 * HEADING:
 *  - By DEFAULT, we treat heading = 0 rad as "intake forward along +X".
 *  - You are free to use ANY heading when requesting poses.
 *  - If your odometry axes/signs differ, DO NOT change this file—flip signs
 *    in your drive/localizer or in how you build actions.
 */
public final class FieldCoordinates {

    private FieldCoordinates() {}

    public static final double TILE = 24.0;
    public static final double HALF_FIELD_TILES = 3.0;   // 6 tiles / 2
    public static final double HALF_TILE = TILE / 2.0;   // 12"

    /**
     * Returns the X coordinate (inches) of the CENTER of the given row (1..6).
     *
     * Row 1 is the bottom of the field (X = -60),
     * Row 6 is the top of the field   (X = +60).
     */
    public static double rowCenterX(int row) {
        if (row < 1 || row > 6) {
            throw new IllegalArgumentException("row must be 1..6");
        }
        // Row indices 1..6 mapped to centers: -60, -36, -12, +12, +36, +60
        return (row - (HALF_FIELD_TILES + 0.5)) * TILE;
    }

    /**
     * Returns the Y coordinate (inches) of the CENTER of the given column (A..F).
     *
     * Column A is the left side of the field (Y = -60),
     * Column F is the right side of the field (Y = +60).
     */
    public static double colCenterY(char col) {
        col = Character.toUpperCase(col);
        if (col < 'A' || col > 'F') {
            throw new IllegalArgumentException("col must be A..F");
        }
        int idx = col - 'A'; // A=0..F=5
        // Col indices 0..5 mapped to centers: -60, -36, -12, +12, +36, +60
        return (idx - (HALF_FIELD_TILES - 0.5)) * TILE;
    }

    /**
     * Tile center pose with an explicit heading (radians).
     * This is PURELY field-centric; heading is whatever you choose.
     */
    public static Pose2d tileCenterPose(char col, int row, double headingRad) {
        return new Pose2d(rowCenterX(row), colCenterY(col), headingRad);
    }

    /**
     * Tile center pose using tile label like "C4".
     */
    public static Pose2d tileCenterPose(String tile, double headingRad) {
        if (tile == null || tile.length() != 2) {
            throw new IllegalArgumentException("tile must look like A1..F6");
        }
        char col = tile.charAt(0);
        int row = Character.getNumericValue(tile.charAt(1));
        return tileCenterPose(col, row, headingRad);
    }

    /**
     * Convenience: tile center with DEFAULT "intake forward" heading (0 rad).
     * Here, 0 rad means "intake pointing along +X".
     */
    public static Pose2d tileCenterIntakeForward(char col, int row) {
        return tileCenterPose(col, row, 0.0);
    }

    public static Pose2d tileCenterIntakeForward(String tile) {
        return tileCenterPose(tile, 0.0);
    }

    // Convenience aliases for common tiles (intake forward by default)
    public static Pose2d C1_IntakeForward() { return tileCenterIntakeForward('C', 1); }
    public static Pose2d C4_IntakeForward() { return tileCenterIntakeForward('C', 4); }
    public static Pose2d A6_IntakeForward() { return tileCenterIntakeForward('A', 6); } // blue goal center
    public static Pose2d F6_IntakeForward() { return tileCenterIntakeForward('F', 6); } // red goal center
}
