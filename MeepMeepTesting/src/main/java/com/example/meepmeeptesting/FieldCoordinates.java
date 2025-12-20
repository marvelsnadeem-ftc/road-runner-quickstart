package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;

public class FieldCoordinates {

    // DECODE field: 144" x 144", origin at field center (0,0).
    public static final double FIELD_SIZE_IN = 144.0;
    public static final double HALF_FIELD_IN = FIELD_SIZE_IN / 2.0; // 72"
    public static final double TILE_SIZE_IN  = 24.0;

    // X-axis (left/right):
    //   Left wall  = -72
    //   Right wall = +72
    // Tile centers (columns A..F) at:
    //   A=-60, B=-36, C=-12, D=+12, E=+36, F=+60
    public static double tileCenterX(int col) {
        // col: 0=A, 1=B, 2=C, 3=D, 4=E, 5=F
        double leftWallX = -HALF_FIELD_IN; // -72
        return leftWallX + TILE_SIZE_IN / 2.0 + col * TILE_SIZE_IN;
    }

    // Y-axis (bottom/top):
    //   Bottom wall = -72 (Row 1)
    //   Top wall    = +72 (Row 6)
    // Tile centers (rows 1..6) at:
    //   Row1=-60, Row2=-36, Row3=-12, Row4=+12, Row5=+36, Row6=+60
    public static double tileCenterY(int row) {
        // row: 0=Row1 (bottom, near audience), 5=Row6 (top, goals)
        double bottomWallY = -HALF_FIELD_IN; // -72
        return bottomWallY + TILE_SIZE_IN / 2.0 + row * TILE_SIZE_IN;
    }

    // Convenience: build Pose2d for tile center with heading
    public static Pose2d tileCenterPose(int col, int row, double headingRadians) {
        return new Pose2d(tileCenterX(col), tileCenterY(row), headingRadians);
    }

    // --------------------------------------------------------------------
    // DECODE GOALS (based on tile centers):
    // Blue goal: tile A6 = (col=0,row=5) → (-60,+60)
    // Red goal:  tile F6 = (col=5,row=5) → (+60,+60)
    // --------------------------------------------------------------------
    public static final Pose2d BLUE_GOAL_CENTER =
            tileCenterPose(0, 5, Math.toRadians(90));  // facing "up" (toward row 6)
    public static final Pose2d RED_GOAL_CENTER  =
            tileCenterPose(5, 5, Math.toRadians(90));  // facing "up"

    // --------------------------------------------------------------------
    // STARTING POSE FOR THIS SIMPLE TEST:
    //
    // You requested: use C1 as the starting position for the robot (Blue alliance)
    // C1 = Column C, Row 1
    // In our index convention:
    //   C  -> col=2
    //   Row1 -> row=0
    // So C1 center = (X=-12, Y=-60)
    // For shooting toward the goals (row 6), heading is +90° (pointing "up" along +Y).
    // --------------------------------------------------------------------
    public static final Pose2d BLUE_START_C1 =
            tileCenterPose(2, 0, Math.toRadians(90));  // C1, facing toward goals

    // For now, we also shoot from this same spot (no driving) to keep it simple.
    public static final Pose2d BLUE_SHOT_POSE_C1 = BLUE_START_C1;
}
