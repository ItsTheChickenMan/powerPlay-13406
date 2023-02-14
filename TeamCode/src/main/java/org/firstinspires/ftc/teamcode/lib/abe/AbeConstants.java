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
	// shoulder related... //
	public static double SHOULDER_RADIUS_INCHES = 3.625;
	public static double SHOULDER_VELOCITY_RADIANS = Math.toRadians(45);

	// slides related... //
	public static double SLIDE_BASE_LENGTH_INCHES = 15.125;
	public static double SLIDES_VELOCITY_INCHES = 16;

	// elbow related... //
	public static double ELBOW_LENGTH_INCHES = 16; // FIXME: update with real value
	public static double ELBOW_VELOCITY_RADIANS = Math.toRadians(90);

	// claw related... //
	public static double CLAW_CLOSED_ANGLE_DEGREES = 10;
	public static double CLAW_OPEN_ANGLE_DEGREES = -90;
}