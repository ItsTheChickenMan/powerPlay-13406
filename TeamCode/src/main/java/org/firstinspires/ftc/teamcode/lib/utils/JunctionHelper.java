package org.firstinspires.ftc.teamcode.lib.utils;

public class JunctionHelper {
	public static final double GROUND_JUNCTION_HEIGHT_INCHES = 0.56;
	public static final double LOW_JUNCTION_HEIGHT = 13.5;
	public static final double MEDIUM_JUNCTION_HEIGHT = 23.5;
	public static final double HIGH_JUNCTION_HEIGHT = 33.5;

	public static enum Level {
		NONE,
		GROUND,
		LOW,
		MEDIUM,
		HIGH
	};

	/**
	 *
	 *
	 * @param x
	 * @param y
	 * @return array of integers, representing the
	 */
	public static int[] rawToSnapped(double x, double y){
		int cx = (int)Math.round(x / 24.0);
		int cy = (int)Math.round(y / 24.0);

		return new int[]{cx, cy};
	}

	public static int[] rawToSnapped(double[] coords){
		return JunctionHelper.rawToSnapped(coords[0], coords[1]);
	}

	public static double[] snappedToRaw(int x, int y){
		return new double[]{x*24.0, y*24.0};
	}

	public static double[] snappedToRaw(int[] coords){
		return JunctionHelper.snappedToRaw(coords[0], coords[1]);
	}

	/**
	 * @brief Takes raw coordinates (in inches) and converts them to the nearest junction position, and then returns the junction level
	 */
	public static JunctionHelper.Level getJunctionLevelFromRaw(double x, double y){
		int[] snapped = JunctionHelper.rawToSnapped(x, y);

		return JunctionHelper.getJunctionLevel(snapped[0], snapped[1]);
	}

	/**
	 * @brief Takes coordinates in multiples of 24 (essentially, tile corner coordinates) and returns the junction level, if applicable
	 */
	public static JunctionHelper.Level getJunctionLevel(int x, int y){
		if((x % 2) == 1 && (y % 2) == 1){
			return JunctionHelper.Level.GROUND;
		} else if( ( (y == 2) || (y == 4) ) && ( (x == 2) || (x == 4) ) ){
			return JunctionHelper.Level.MEDIUM;
		} else if( ( y == 3 && (x == 2 || x == 4) ) || (x == 3 && (y == 2 || y == 4) ) ){
			return JunctionHelper.Level.HIGH;
		} else if( ( (y == 1 || y == 5) && (x == 2 || x == 4) ) || ( (y == 2 || y == 4) && (x == 1 || x == 5) ) ){
			return JunctionHelper.Level.LOW;
		} else {
			return JunctionHelper.Level.NONE;
		}
	}

	public static double getJunctionHeightFromRaw(double x, double y){
		int[] snapped = JunctionHelper.rawToSnapped(x, y);

		return JunctionHelper.getJunctionHeight(snapped[0], snapped[1]);
	}

	public static double getJunctionHeight(int x, int y){
		Level level = JunctionHelper.getJunctionLevel(x, y);

		switch(level){
			case GROUND:
				return JunctionHelper.GROUND_JUNCTION_HEIGHT_INCHES;
			case LOW:
				return JunctionHelper.LOW_JUNCTION_HEIGHT;
			case MEDIUM:
				return JunctionHelper.MEDIUM_JUNCTION_HEIGHT;
			case HIGH:
				return JunctionHelper.HIGH_JUNCTION_HEIGHT;
			case NONE:
			default:
				return 0.0;
		}
	}
}
