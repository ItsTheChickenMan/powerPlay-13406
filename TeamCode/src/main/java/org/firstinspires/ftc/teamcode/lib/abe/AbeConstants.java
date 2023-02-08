package org.firstinspires.ftc.teamcode.lib.abe;

import org.firstinspires.ftc.teamcode.lib.motion.PositionableMotor;
import org.firstinspires.ftc.teamcode.lib.utils.GlobalStorage;
import org.firstinspires.ftc.teamcode.lib.utils.JunctionHelper;

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

	public static final double DRIVE_AIM_P_CONSTANT = 95;
	public static final double DRIVE_AIM_I_CONSTANT = 0;
	public static final double DRIVE_AIM_D_CONSTANT = 0;
	//public static final double DRIVE_EXTRA_KICK = 8.0;

	/*
	public static final double DRIVE_AIM_RATE_OF_INCREASE = 2;
	public static final double DRIVE_DIAGONAL_RADIUS = Math.sqrt(6.25*6.25 + 5.5*5.5);
	*/

	// generally arm related... //
	//public static final double ARM_LATERAL_OFFSET_INCHES = 2.4375;
	public static final double ARM_LATERAL_OFFSET_INCHES = 2.1;
	public static final double ARM_LONGINAL_OFFSET_INCHES = -5.0;
	public static final double ARM_VERTICAL_OFFSET_INCHES = 4.625;
	public static final double ARM_POLE_HEIGHT_OFFSET_INCHES = 0.0; // how many inches to aim above a pole

	// how far the arm falls per inch at 0 degrees
	// multiplied by cos(angle) to determine expected angular adjustment required at a given extension
	//public static final double ANGULAR_FALLOFF_PER_INCH_RADIANS = Math.toRadians(-4.0) / 30.0;
	//public static final double ANGULAR_FALLOFF_PER_INCH_RADIANS = Math.toRadians(-0.0) / 30.0;

	//public static final double HEIGHT_FALLOFF_PER_INCH_INCHES = -0.11439;

	// sag correction constants

	// TUNING
	// (To correct for sag, the AbeArm class computes the expected torque that the arm applies to the elbow's point of rotation and then accounts for it with the ANGULAR_CORRECTION_PER_KG_IN value)
	//	-"Zero point" is the angle of the elbow at which there is no torque on the elbow.  This varies depending on the extension of the slides.
	//	-Increasing ANGULAR_CORRECTION_PER_KG_IN will increase the angular correction, but has no effect on the zero point.=
	//	-Increasing ARM_MASS_LEFT_KG will increase the angular correction and generally increase the zero point
	//	-Increasing ARM_MASS_RIGHT_KG will increase the angular correction and generally decrease the zero point
	//	-Increasing ELBOW_MASS_KG will decrease the angular correction and generally decrease the zero point
	//	-Increasing TRUE_ELBOW_RADIUS_INCHES will increase the angular correction and generally increase the zero point (BUT this should stay constant)
	//	-Increasing ARM_OFFSET_RIGHT_INCHES will increase the angular correction and generally decrease the zero point (BUT this should stay constant)
	//	-Increasing TORQUE_INTERCEPT will linearly increase correction.  Although not technically physically correct, it helps at times when the correction seems to be off by a constant amount.
	//	-You should generally only tune the following constants:
	//		-ARM_MASS_LEFT_KG
	//		-ARM_MASS_RIGHT_KG
	//		-ELBOW_MASS_KG
	//		-ANGULAR_CORRECTION_PER_KG_IN
	//		-TORQUE_INTERCEPT
	//	-The others should just be physically measured and kept constant unless a new, more accurate measurement is found.

	// TUNE THESE!
	public static final double ANGULAR_CORRECTION_PER_KG_IN = 0.00192; // units in radians / (kg*in)

	public static final double ARM_MASS_LEFT_KG = 1.56;
	public static final double ARM_MASS_RIGHT_KG = 0.497;
	public static final double ELBOW_MASS_KG = 0.1;
	public static final double TORQUE_INTERCEPT = 1.0;

	// DON'T TUNE THESE!
	public static final double ARM_OFFSET_RIGHT_INCHES = 2.75;
	public static final double TRUE_ELBOW_RADIUS_INCHES = 1.75;

	public static double getPrecalculatedElbowSagAtLevel(double distance, JunctionHelper.Level level){
		// 0, 16 = 0.063944
		// 0, 22 = 0.080398
		// 0, 28 = 0.098994
		// 0, 34 = 0.122801

		// 1, 16 = 0.103697
		// 1, 22 = 0.112206
		// 1, 28 = 0.127968
		// 1, 34 = 0.147778

		// 2, 16 = 0.136497
		// 2, 22 = 0.139939
		// 2, 28 = 0.155222
		// 2, 34 = 0.174956

		// 3, 16 = 0.192951
		// 3, 22 = 0.175684
		// 3, 28 = 0.189580
		// 3, 34 = 0.200245

		// ----------------

		// 0, 16 = 0.0
		// 0, 22 = 0.0653144
		// 0, 28 = 0.0759409
		// 0, 34 = 0.0979446

		// 1, 16 = 0.0763664
		// 1, 22 = 0.0863347
		// 1, 28 = 0.1002759
		// 1, 34 = 0.1139091

		// 2, 16 = 0.1034603
		// 2, 22 = 0.1046031
		// 2, 28 = 0.1260978
		// 2, 34 = 0.1329843

		// 3, 16 = 0.1553812
		// 3, 22 = 0.1553812
		// 3, 28 = 0.1459976
		// 3, 34 = 0.1607509

		// --------------------

		// 0, 16 = 0.01175755
		// 0, 22 = 0.08010008
		// 0, 28 = 0.09315544
		// 0, 34 = 0.11543916

		// 1, 16 = 0.08679404
		// 1, 22 = 0.09887151
		// 1, 28 = 0.11701653
		// 1, 34 = 0.13414592

		// 2, 16 = 0.10969759
		// 2, 22 = 0.11280045
		// 2, 28 = 0.13429005
		// 2, 34 = 0.14516762

		// 3, 16 = 0.15189753
		// 3, 22 = 0.14560676
		// 3, 28 = 0.15419674
		// 3, 34 = 0.17711154

		// 3, 09 = 0.09570892
		// 3, 15 = 0.11806050
		// 3, 21 = 0.11642312
		// 3, 27 = 0.14596395

		// -------
		// 1, 16: 0.059594647
		// 1, 22: 0.086850566
		// 1, 28: 0.098710917
		// 1, 34: 0.111407657

		// 2, 16: 0.061812062
		// 2, 22: 0.077301832
		// 2, 28: 0.095631629
		// 2, 34: 0.111018085

		// 3, 09:-0.02021266
		// 3, 15: 0.03559414
		// 3, 21: 0.06710116
		// 3, 27: 0.08947199

		double correction = 0.0;

		// pre-calculated using measured values (using ArmTuner) and quadratic regression
		if(level == JunctionHelper.Level.LOW){
			//correction = 0.00007847916667*distance*distance - 0.0014572083*distance + 0.1067614167;
			//correction = 0.00004*distance*distance + 0.07;
			//correction = 0.00003508277778*distance*distance + 0.0009158721111*distance + 0.0628047361;
			correction = -0.00010110540972222*distance*distance + 0.00784359350277*distance - 0.03920826630555;
		} else if(level == JunctionHelper.Level.MEDIUM){
			//correction = 0.0001131388889*distance*distance - 0.0034792778*distance + 0.1628323889;
			//correction = 0.000055*distance*distance - 0.0006*distance + 0.095;
			//correction = 0.00005399104167*distance*distance - 0.0005678905833*distance + 0.1035121942;
			correction = 0.0027657978*distance + 0.0172959578;
		} else if(level == JunctionHelper.Level.HIGH) {
			//correction = 0.0001939722222*distance*distance - 0.0091023111*distance + 0.2872113889;
			//correction = 0.00011*distance*distance - 0.00617*distance + 0.24;
			correction = 0.000011700300925913*distance*distance*distance - 0.00086401048611041*distance*distance + 0.02487755229165*distance - 0.18265530062495;
			/*if(distance > 27){
				correction = 0.0002028164583*distance*distance - 0.0087369561*distance + 0.2397400175;
			} else {
				correction = 0.00004256726080247*distance*distance*distance - 0.00224870673611*distance*distance  + 0.03892206298611*distance - 0.10347593437501;
			}*/
		} else if(level == JunctionHelper.Level.GROUND) {
			//correction = 0.0000510625*distance*distance + 0.00069965833*distance + 0.0398309167;
			//correction = 0.00008*distance*distance + 0.0005*distance;
			correction = 0.00008460861111*distance*distance - 0.0017408832*distance + 0.0765079811;
		}

		correction = Math.min(correction, Math.toRadians(13.5));

		return correction;
	}

	// 0 = ground
	// 1 = low
	// 2 = mid
	// 3 = high
	public static double getPrecalculatedElbowSag(double distance, double height){
		double correction = 0.0;

		JunctionHelper.Level lowLevel = JunctionHelper.Level.GROUND;
		JunctionHelper.Level highLevel = JunctionHelper.Level.NONE;

		// get two enveloping heights
		for(JunctionHelper.Level level : JunctionHelper.JUNCTION_LEVELS){
			double h = JunctionHelper.getJunctionHeight(level);

			if(height >= h) lowLevel = level;
			if(height <= h){
				highLevel = level;
				break;
			}
		}

		if(highLevel == JunctionHelper.Level.NONE){
			return 0.0;
		}

		// get low and high correction
		double lowCorrection = AbeConstants.getPrecalculatedElbowSagAtLevel(distance, lowLevel);
		double highCorrection = AbeConstants.getPrecalculatedElbowSagAtLevel(distance, highLevel);

		//GlobalStorage.globalTelemetry.addData("lowCorrection", lowCorrection);
		//GlobalStorage.globalTelemetry.addData("highCorrection", highCorrection);

		// skip calculations if they're the same
		if(lowLevel == highLevel){
			return lowCorrection;
		}

		// get height differences between correction junctions
		double lowHeight = JunctionHelper.getJunctionHeight(lowLevel);
		double highHeight = JunctionHelper.getJunctionHeight(highLevel);

		// map aim height to percentage
		double percentage = (height - lowHeight) / (highHeight - lowHeight);

		//GlobalStorage.globalTelemetry.addData("percentage", percentage);

		// calculate
		return percentage*(highCorrection - lowCorrection) + lowCorrection;
	}

	/*
	public static double getExpectedElbowSag(double slideExtension){
		// approximately linear relationship
		// determined through testing at zero degrees
		// works best <30 in, outside of that it tends to fail due to non linearity of sagging at that range
		//return slideExtension*0.0039850756 - 0.0211473332;
		//return slideExtension*0.0025920267 + 0.0033715571;
		return slideExtension*0.0036577917 - 0.0332507529;
	}*/

	// elbow related... //
	public static final double ELBOW_GEAR_RATIO = 24.0; // 24:1
	public static final double ELBOW_TICK_RATIO = 384.5; // ticks/rotation
	public static final double ELBOW_LOWER_LIMIT_ROTATIONS = PositionableMotor.degreesToRotations(-10);
	public static final double ELBOW_UPPER_LIMIT_ROTATIONS = PositionableMotor.degreesToRotations(75);
	public static final double ELBOW_RADIUS_INCHES = 3.625; // how far the slides are from the rotation point

	// slides related... //
	public static final double SLIDE_GEAR_RATIO = 1; // 1:1
	public static final double SLIDE_TICK_RATIO = 384.5;
	public static final double SLIDE_SPOOL_CIRCUMFERENCE_INCHES = 4.40944882;
	public static final double SLIDE_BASE_LENGTH_INCHES = 15.125;
	public static final double SLIDE_MAX_EXTENSION_INCHES = 38; // leaving off two inches as a just-in-case pillow...
	public static final double SLIDE_EXTENSION_FACTOR = 1.00; // this could probably just be factored into the circumference, but I've already implemented this so I'm not worried about it

	// wrist related... //
	public static final double WRIST_BASE_ANGLE_DEGREES = 0;
	public static final double WRIST_MAX_RANGE_DEGREES = 295;
	public static final double WRIST_MAX_RANGE_RADIANS = Math.toRadians(AbeConstants.WRIST_MAX_RANGE_DEGREES);
	public static final double WRIST_OFFSET_INCHES = 5.85; // how far forward the wrist hole is from the end of the slides
	//public static final double WRIST_OFFSET_INCHES = 0.0; // how far forward the wrist hole is from the end of the slides
	public static final double WRIST_HOLDING_ANGLE_DEGREES = -45.0;
	public static final double WRIST_DEPOSITING_ANGLE_DEGREES = 0.0;

	// fingers related... //
	public static final double FINGERS_MAX_RANGE_DEGREES = 360; // FIXME: update value
	public static final double FINGERS_MAX_RANGE_RADIANS = Math.toRadians(AbeConstants.FINGERS_MAX_RANGE_DEGREES);
	public static final double FINGERS_CLOSED_POSITION = 10;
	public static final double FINGERS_OPEN_POSITION = -90;
}
