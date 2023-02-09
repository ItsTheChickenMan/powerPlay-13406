package org.firstinspires.ftc.teamcode.lib.utils;

public class JunctionHelper {
	public static final double FIELD_OFFSET = 0.0;
	public static final double FIELD_SCALE = 23.5;
	public static final double FIELD_WIDTH = 141;
	public static final double FIELD_LENGTH = 141;

	public static final double NONE_JUNCTION_HEIGHT_INCHES = -1.0;
	public static final double GROUND_JUNCTION_HEIGHT_INCHES = 0.56;
	public static final double LOW_JUNCTION_HEIGHT_INCHES = 13.5;
	public static final double MEDIUM_JUNCTION_HEIGHT_INCHES = 23.5;
	public static final double HIGH_JUNCTION_HEIGHT_INCHES = 33.5;

	public static final Level[] JUNCTION_LEVELS = {Level.GROUND, Level.LOW, Level.MEDIUM, Level.HIGH};
	public static final double[] JUNCTION_HEIGHTS = {JunctionHelper.GROUND_JUNCTION_HEIGHT_INCHES, JunctionHelper.LOW_JUNCTION_HEIGHT_INCHES, JunctionHelper.MEDIUM_JUNCTION_HEIGHT_INCHES, JunctionHelper.HIGH_JUNCTION_HEIGHT_INCHES};

	public static enum Level {
		NONE,
		GROUND,
		LOW,
		MEDIUM,
		HIGH
	};

	/**
	 * @brief Checks if a height returned by the class is valid or not
	 *
	 * @param height
	 * @return
	 */
	public static boolean isValidHeight(double height){
		return height >= 0.0;
	}

	/**
	 * @param x
	 * @param y
	 * @return array of integers w/ coords
	 */
	public static int[] rawToSnapped(double x, double y){
		int cx = (int)Math.round((x - JunctionHelper.FIELD_OFFSET) / JunctionHelper.FIELD_SCALE);
		int cy = (int)Math.round((y - JunctionHelper.FIELD_OFFSET) / JunctionHelper.FIELD_SCALE);

		return new int[]{cx, cy};
	}

	public static int[] rawToSnapped(double[] coords){
		return JunctionHelper.rawToSnapped(coords[0], coords[1]);
	}

	public static double[] snappedToRaw(int x, int y){
		return new double[]{x*JunctionHelper.FIELD_SCALE + JunctionHelper.FIELD_OFFSET, y*JunctionHelper.FIELD_SCALE + JunctionHelper.FIELD_OFFSET};
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
	 * @brief Takes coordinates in multiples of FIELD_SCALE (essentially, tile corner coordinates) and returns the junction level, if applicable
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

	public static double getJunctionHeight(JunctionHelper.Level level){
		switch(level){
			case GROUND:
				return JunctionHelper.GROUND_JUNCTION_HEIGHT_INCHES;
			case LOW:
				return JunctionHelper.LOW_JUNCTION_HEIGHT_INCHES;
			case MEDIUM:
				return JunctionHelper.MEDIUM_JUNCTION_HEIGHT_INCHES;
			case HIGH:
				return JunctionHelper.HIGH_JUNCTION_HEIGHT_INCHES;
			case NONE:
			default:
				return JunctionHelper.NONE_JUNCTION_HEIGHT_INCHES;
		}
	}

	public static double getJunctionHeight(int x, int y){
		Level level = JunctionHelper.getJunctionLevel(x, y);

		return JunctionHelper.getJunctionHeight(level);
	}

	private static boolean approximatelyEqual(double a, double b){
		return Math.abs(a - b) < 1.0;
	}

	public static JunctionHelper.Level rawHeightToJunctionHeight(double height){
		if( JunctionHelper.approximatelyEqual(height, JunctionHelper.GROUND_JUNCTION_HEIGHT_INCHES) ){
			return Level.GROUND;
		} else if( JunctionHelper.approximatelyEqual(height, JunctionHelper.LOW_JUNCTION_HEIGHT_INCHES) ){
			return Level.LOW;
		} else if( JunctionHelper.approximatelyEqual(height, JunctionHelper.MEDIUM_JUNCTION_HEIGHT_INCHES) ){
			return Level.MEDIUM;
		} else if( JunctionHelper.approximatelyEqual(height, JunctionHelper.HIGH_JUNCTION_HEIGHT_INCHES) ){
			return Level.HIGH;
		}

		return Level.NONE;
	}
}
