package org.firstinspires.ftc.teamcode.lib.abe;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Vector2d;

/**
 * @brief Contains all constant values required for Abe functionality.  All bot characteristics and settings should be here.
 *
 * Generally speaking, configurable constants should only be in here, and should only be used in classes in the abe package.
 */

// make configurable from dashboard
@Config
public class AbeConstants {
	// general... //

	// drive related... //
	public static PIDCoefficients AIM_HEADING_PID = new PIDCoefficients(5.5, 0, 0.2);
	public static double MAX_ROTATION_POWER = 0.6;

	// general arm related... //
	public static double ARM_X_OFFSET_INCHES = -5; // offset along robot's forward axis
	public static double ARM_Y_OFFSET_INCHES = 4.625; // offset along robot's up axis
	public static double ARM_Z_OFFSET_INCHES = -1.75; // offset along robot's perpendicular axis (to forward)

	// tuning
	// the output of ArmTuner.java can be copied straight into the body of this array
	public static Vector2d[][] ARM_TUNING_POINTS = new Vector2d[][]{
					// GROUND
					{new Vector2d(15.091773610983092, 0.05219463456884287), new Vector2d(19.33499270935176, 0.07099615623484204), new Vector2d(23.978207713600327, 0.08262834195745368), new Vector2d(28.3197508655995, 0.09746804585375188), new Vector2d(31.75924207279625, 0.10585026684151667), new Vector2d(34.18289307979259, 0.12268163503441652), new Vector2d(40.194086404572566, 0.1428522377056018), new Vector2d(46.622598407274594, 0.16792959510155125), },

					// LOW
					{new Vector2d(12.48494042152325, 0.042544272945434974), new Vector2d(18.266475176415938, 0.042544272945434974), new Vector2d(24.39592420926758, 0.07206609757318491), new Vector2d(27.578200645172934, 0.07798684111992168), new Vector2d(33.235506469404214, 0.10334989950579139), new Vector2d(40.0286439132881, 0.12687605984514477), new Vector2d(45.53626401497838, 0.15165166326446242), new Vector2d(50.64124088651014, 0.1820427731297932), },

					// MEDIUM
					{new Vector2d(7.7930629329156345, 0.008719066912313162), new Vector2d(17.55867924579155, 0.039166418608944306), new Vector2d(26.555436663368287, 0.07558560915412624), new Vector2d(33.40675035239006, 0.09661378701204344), new Vector2d(38.78066033992776, 0.12906099587969483), new Vector2d(45.046701628524175, 0.1534837947748453), new Vector2d(47.73404903516332, 0.1740937642144236), },

					// HIGH
					{new Vector2d(4.746460100904496, -0.03525836594002868), new Vector2d(10.02331844680328, -0.006963713463308633), new Vector2d(7.418752014292681, -0.015437460261931974), new Vector2d(19.12705449082631, 0.04470849225556846), new Vector2d(26.96909517842078, 0.06762165230067604), new Vector2d(32.706758838686234, 0.09299150937919991), new Vector2d(38.133377139231214, 0.1135830228658838), new Vector2d(42.876798737639, 0.14291454064554776), },
	};

	// elbow related... //
	public static double ELBOW_TPR = 384.5;
	public static double ELBOW_GEAR_RATIO = 24;
	public static double ELBOW_RADIUS_INCHES = 3.625;
	public static double ELBOW_VELOCITY_RADIANS = Math.toRadians(50);
	public static double ELBOW_LOWER_LIMIT_RADIANS = Math.toRadians(-16);
	public static double ELBOW_UPPER_LIMIT_RADIANS = Math.toRadians(75);
	public static double ELBOW_INITIALIZATION_ANGLE_RADIANS = Math.toRadians(26.45);

	// slides related... //
	public static double SLIDES_TPR = 384.5;
	public static double SLIDES_GEAR_RATIO = 1;
	public static double SLIDES_BASE_LENGTH_INCHES = 14.5;
	public static double SLIDES_MAX_EXTENSION_INCHES = 38;
	public static double SLIDES_VELOCITY_INCHES = 36;
	public static double SLIDES_SPOOL_CIRCUMFERENCE_INCHES = 4.409;

	// wrist related... //
	public static double WRIST_MAX_RANGE_DEGREES = 245;
	public static double WRIST_ZERO_ANGLE_DEGREES = -60.0;
	public static double WRIST_OFFSET_INCHES = 5.8;
	public static double WRIST_DEFAULT_ANGLE_DEGREES = -80.0;
	public static double WRIST_UP_ANGLE_DEGREES = 45.0;
	public static double WRIST_HIGH_ANGLE_DEGREES = 55;
	public static double WRIST_DROP_ANGLE_DEGREES = 0.0;
	public static double WRIST_GRAB_ANGLE_DEGREES = -10.0;

	// claw related... //
	public static double CLAW_MAX_RANGE_DEGREES = 300;
	public static double CLAW_ZERO_ANGLE_DEGREES = 0.0;
	public static double CLAW_CLOSED_ANGLE_DEGREES = 90;
	public static double CLAW_OPEN_ANGLE_DEGREES = 0;
}