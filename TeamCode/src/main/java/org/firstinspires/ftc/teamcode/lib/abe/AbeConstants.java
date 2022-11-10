package org.firstinspires.ftc.teamcode.lib.abe;

/**
 * @brief Helpful constants which describe particular unchanging aspects of the robot.
 *
 * There is no point in constructing this class, as every member of it should be static.
 *
 * Names should be very self-documenting (but not overly lengthy)
 */
public class AbeConstants {
	// drive related... //
	public static final double DRIVE_GEAR_RATIO = 1; // 1:1
	public static final double DRIVE_TICK_RATIO = 384.5; // 384.5 ticks/rotation
	public static final double DRIVE_WHEEL_RADIUS_INCHES = 1.88976378;

	// elbow related... //
	public static final double ELBOW_GEAR_RATIO = 24.0; // 24:1
	public static final double ELBOW_TICK_RATIO = 751.8; // 751.8 ticks/rotation
	public static final double ELBOW_LOWER_LIMIT_DEGREES = 10;
	public static final double ELBOW_UPPER_LIMIT_DEGREES = -50;

	// slides related... //
	public static final double SLIDE_GEAR_RATIO = 1; // 1:1
	public static final double SLIDE_TICK_RATIO = 384.5;
	public static final double SLIDE_SPOOL_RADIUS_INCHES = 0.701785576; // inches
	public static final double SLIDE_BASE_LENGTH_INCHES = 18;
	public static final double SLIDE_MAX_EXTENSION_INCHES = 38; // leaving off two inches as a just-in-case pillow...

	// wrist related... //
	public static final double WRIST_MAX_RANGE_DEGREES = 270;
	public static final double WRIST_MAX_RANGE_RADIANS = Math.toRadians(AbeConstants.WRIST_MAX_RANGE_DEGREES);

	// fingers related... //
	public static final double FINGERS_MAX_RANGE_DEGREES = 360; // FIXME: update value
	public static final double FINGERS_MAX_RANGE_RADIANS = Math.toRadians(AbeConstants.FINGERS_MAX_RANGE_DEGREES);
	public static final double FINGERS_CLOSED_POSITION = -115;
	public static final double FINGERS_OPEN_POSITION = 45;
}
