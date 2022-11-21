package org.firstinspires.ftc.teamcode.lib.abe;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.motion.PositionableMotor;

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

	public static final double AIM_P_CONSTANT = 1.0; // 0.75
	public static final double AIM_I_CONSTANT = 0.0; // idk whatever
	public static final double AIM_D_CONSTANT = 0.035; // 0.025

	// generally arm related... //
	public static final double ARM_LATERAL_OFFSET_INCHES = 2.1;
	public static final double ARM_LONGINAL_OFFSET_INCHES = -4.9;
	public static final double ARM_VERTICAL_OFFSET_INCHES = 4.75;

	// how far the arm falls per inch at 0 degrees
	// multiplied by cos(angle) to determine expected angular adjustment required at a given extension
	public static final double ANGULAR_FALLOFF_PER_INCH_RADIANS = Math.toRadians(-4.0) / 30.0;

	// elbow related... //
	public static final double ELBOW_GEAR_RATIO = 24.0; // 24:1
	public static final double ELBOW_TICK_RATIO = 751.8; // 751.8 ticks/rotation
	public static final double ELBOW_LOWER_LIMIT_ROTATIONS = PositionableMotor.degreesToRotations(-10);
	public static final double ELBOW_UPPER_LIMIT_ROTATIONS = PositionableMotor.degreesToRotations(70);
	public static final double ELBOW_RADIUS_INCHES = 4.4; // how far the slides are from the rotation point

	// slides related... //
	public static final double SLIDE_GEAR_RATIO = 1; // 1:1
	public static final double SLIDE_TICK_RATIO = 384.5;
	public static final double SLIDE_SPOOL_CIRCUMFERENCE_INCHES = 4.40944882; // inches
	public static final double SLIDE_BASE_LENGTH_INCHES = 15;
	public static final double SLIDE_MAX_EXTENSION_INCHES = 38; // leaving off two inches as a just-in-case pillow...
	public static final double SLIDE_OFFSET_INCHES = -2; // how far off the slides are from the rotation point

	// wrist related... //
	public static final double WRIST_MAX_RANGE_DEGREES = 270;
	public static final double WRIST_MAX_RANGE_RADIANS = Math.toRadians(AbeConstants.WRIST_MAX_RANGE_DEGREES);
	public static final double WRIST_OFFSET_INCHES = 6.4; // how far forward the wrist hole is from the end of the slides

	// fingers related... //
	public static final double FINGERS_MAX_RANGE_DEGREES = 360; // FIXME: update value
	public static final double FINGERS_MAX_RANGE_RADIANS = Math.toRadians(AbeConstants.FINGERS_MAX_RANGE_DEGREES);
	public static final double FINGERS_CLOSED_POSITION = -115;
	public static final double FINGERS_OPEN_POSITION = 45;
}
