package org.firstinspires.ftc.teamcode.lib.utils;

/**
 * @brief Contains random methods for doing things with angles
 */
public class AngleHelper {
	/**
	 * @brief Snaps angle between -pi and pi
	 *
	 * This will never return a number greater than pi, because a rotation greater than pi is the same as a lesser rotation in the negative direction
	 * This will never return a number less than -pi, because a rotation less than -pi is the same as a lesser rotation in the positive direction
	 *
	 * @param a the angle to normalize, in radians
	 * @return the normalized angle, in radians
	 */
	public static double normalizeAngleRadians(double a){
		// snap angle between 0 and pi*2
		a %= Math.PI*2;

		return a>Math.PI ? a-Math.PI*2 : (a<-Math.PI ? Math.PI*2 + a : a);
	}

	/**
	 * @brief Snaps angle between -180 and 180
	 *
	 * This will never return a number greater than 180, because a rotation greater than 180 is the same as a lesser rotation in the negative direction
	 * This will never return a number less than -180, because a rotation less than -180 is the same as a lesser rotation in the positive direction
	 *
	 * @param a
	 * @return
	 */
	public static double normalizeAngleDegrees(double a){
		// snap angle between 0 and 360
		a %= 360;

		return a>180 ? a-360 : (a<-180 ? 360+a : a);
	}

	/**
	 * @brief Returns the shortest possible angular adjustment required to make a1 essentially equal to a2
	 *
	 * @param a1 the second angle in a2 - a1, in radians
	 * @param a2 the first angle in a2 - a1, in radians
	 * @return the minimum change required for a1 to make a1 essentially equal to a2, in radians
	 */
	public static double angularDistanceRadians(double a1, double a2){
		return normalizeAngleRadians(a2 - a1);
	}

	/**
	 * @brief Returns the shortest possible angular adjustment required to make a1 essentially equal to a2
	 *
	 * @param a1 the second angle in a2 - a1, in degrees
	 * @param a2 the first angle in a2 - a1, in degrees
	 * @return the minimum change required for a1 to make a1 essentially equal to a2, in degrees
	 */
	public static double angularDistanceDegrees(double a1, double a2){
		return normalizeAngleDegrees(a2 - a1);
	}
}
