package org.firstinspires.ftc.teamcode.lib.abe;

import org.firstinspires.ftc.teamcode.lib.motion.PositionableMotor;

/**
 * @brief Helpful constants which describe particular unchanging aspects of the robot.
 *
 * There is no point in constructing this class, as every member of it should be static.
 *
 * Names should be very self-documenting
 */
public class AbeConstants {
	// drive related... //
	public static final double DRIVE_GEAR_RATIO = 1; // 1:1
	public static final double DRIVE_TICK_RATIO = 384.5; // 384.5 ticks/rotation

	public static final double DRIVE_WHEEL_CIRCUMFERENCE_INCHES = 1.88976378 * Math.PI * 2;
	public static final double DRIVE_MAX_VELOCITY = 36; // inches/second

	public static final double DRIVE_AIM_P_CONSTANT = 150;
	public static final double DRIVE_AIM_I_CONSTANT = 0;
	public static final double DRIVE_AIM_D_CONSTANT = 5;

	/*
	public static final double DRIVE_AIM_RATE_OF_INCREASE = 2;
	public static final double DRIVE_DIAGONAL_RADIUS = Math.sqrt(6.25*6.25 + 5.5*5.5);
	*/

	// generally arm related... //
	public static final double ARM_LATERAL_OFFSET_INCHES = 2.125;
	public static final double ARM_LONGINAL_OFFSET_INCHES = -5.125;
	public static final double ARM_VERTICAL_OFFSET_INCHES = 4.75;
	public static final double ARM_POLE_HEIGHT_OFFSET_INCHES = 6.0; // how many inches to aim above a pole

	// how far the arm falls per inch at 0 degrees
	// multiplied by cos(angle) to determine expected angular adjustment required at a given extension
	//public static final double ANGULAR_FALLOFF_PER_INCH_RADIANS = Math.toRadians(-4.0) / 30.0;
	//public static final double ANGULAR_FALLOFF_PER_INCH_RADIANS = Math.toRadians(-0.0) / 30.0;

	//public static final double HEIGHT_FALLOFF_PER_INCH_INCHES = -0.11439;

	public static final double ELBOW_TORQUE_FACTOR = 2.5;

	public static double getExpectedElbowSag(double slideExtension){
		// approximately linear relationship
		// determined through testing at zero degrees
		// works best within a 1.5 tile range, outside of that it tends to fail due to non linearity of sagging at that range
		return slideExtension*0.0039850756 - 0.0211473332;
	}

	// elbow related... //
	public static final double ELBOW_GEAR_RATIO = 24.0; // 24:1
	public static final double ELBOW_TICK_RATIO = 751.8; // 751.8 ticks/rotation
	public static final double ELBOW_LOWER_LIMIT_ROTATIONS = PositionableMotor.degreesToRotations(-10);
	public static final double ELBOW_UPPER_LIMIT_ROTATIONS = PositionableMotor.degreesToRotations(70);
	public static final double ELBOW_RADIUS_INCHES = 4.4; // how far the slides are from the rotation point

	// slides related... //
	public static final double SLIDE_GEAR_RATIO = 1; // 1:1
	public static final double SLIDE_TICK_RATIO = 384.5;
	public static final double SLIDE_SPOOL_CIRCUMFERENCE_INCHES = 4.40944882;
	public static final double SLIDE_BASE_LENGTH_INCHES = 16;
	public static final double SLIDE_MAX_EXTENSION_INCHES = 38; // leaving off two inches as a just-in-case pillow...
	public static final double SLIDE_EXTENSION_FACTOR = 1.0875; // this could probably just be factored into the circumference, but I've already implemented this so I'm not worried about it

	/**
	 * @deprecated just adjust the base length instead
	 */
	public static final double SLIDE_OFFSET_INCHES = -1.5; // how far off the slides are from the rotation point

	// wrist related... //
	public static final double WRIST_MAX_RANGE_DEGREES = 295;
	public static final double WRIST_MAX_RANGE_RADIANS = Math.toRadians(AbeConstants.WRIST_MAX_RANGE_DEGREES);
	public static final double WRIST_OFFSET_INCHES = 6.5; // how far forward the wrist hole is from the end of the slides

	// fingers related... //
	public static final double FINGERS_MAX_RANGE_DEGREES = 360; // FIXME: update value
	public static final double FINGERS_MAX_RANGE_RADIANS = Math.toRadians(AbeConstants.FINGERS_MAX_RANGE_DEGREES);
	public static final double FINGERS_CLOSED_POSITION = -35;
	public static final double FINGERS_OPEN_POSITION = -135;
}
