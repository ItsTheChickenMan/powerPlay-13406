package org.firstinspires.ftc.teamcode.lib.abe;

import com.acmerobotics.dashboard.config.Config;

/**
 * @brief Contains all constant values required for Abe functionality.  All bot characteristics and settings should be here.
 *
 * Generally speaking, configurable constants should only be in here, and should only be used in classes in the abe package.
 */

// make configurable from dashboard
@Config
public class AbeConstants {
	// general... //
	public static double[] JUNCTION_HEIGHT_OFFSETS_INCHES = new double[]{6.0, 2.0, 0.0, 0.0}; // ground, low, medium, high

	// general arm related... //
	public static double ARM_X_OFFSET_INCHES = -5; // rr coords
	public static double ARM_Y_OFFSET_INCHES = -2.1;
	public static double ARM_Z_OFFSET_INCHES = 4.625;

	// shoulder related... //
	public static double SHOULDER_TPR = 1425.1;
	public static double SHOULDER_GEAR_RATIO = 1;
	public static double SHOULDER_RADIUS_INCHES = 3.625;
	public static double SHOULDER_VELOCITY_RADIANS = Math.toRadians(45);

	// slides related... //
	public static double SLIDES_TPR = 384.5;
	public static double SLIDES_GEAR_RATIO = 1;
	public static double SLIDES_BASE_LENGTH_INCHES = 15.125;
	public static double SLIDES_MAX_LENGTH_INCHES = 35; // FIXME: update
	public static double SLIDES_VELOCITY_INCHES = 16;
	public static double SLIDES_SPOOL_CIRCUMFERENCE_INCHES = 4.409;

	// elbow related... //
	public static double ELBOW_TPR = 384.5;
	public static double ELBOW_GEAR_RATIO = 28;
	public static double ELBOW_LENGTH_INCHES = 16; // FIXME: update with real value
	public static double ELBOW_VELOCITY_RADIANS = Math.toRadians(90);

	// wrist related... //
	public static double WRIST_OFFSET_INCHES = 12;

	// claw related... //
	public static double CLAW_CLOSED_ANGLE_DEGREES = 10;
	public static double CLAW_OPEN_ANGLE_DEGREES = -90;
}