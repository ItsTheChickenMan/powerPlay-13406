package org.firstinspires.ftc.teamcode.lib.utils;

public class MathUtils {
	public static boolean approximatelyEqual(double a, double b, double range){
		return Math.abs(a - b) <= range;
	}

	/**
	 * @brief map a value (val) from between l1 and h1 to between l2 and h2.
	 *
	 * ex: val = 1, l1 = 0, h1 = 2, l2 = 2, h2 = 4, output = 3
	 *
	 * @param val
	 * @param l1
	 * @param h1
	 * @param l2
	 * @param h2
	 * @return
	 */
	public static double map(double val, double l1, double h1, double l2, double h2){
		double interp = (val - l1) / (h1 - l1);

		double out = interp * (h2 - l2) + l2;

		return out;
	}
}
