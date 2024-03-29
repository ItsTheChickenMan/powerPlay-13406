package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    ^
 *    |
 *    | ( x direction)
 *    |
 *    v
 *    <----( y direction )---->

 *        (forward)
 *    /--------------\
 *    |     ____     |
 *    |     ----     |    <- Perpendicular Wheel
 *    |           || |
 *    |           || |    <- Parallel Wheel
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
public class TwoWheelTrackingLocalizer extends TwoTrackingWheelLocalizer {
	public static final int LEFT = 1;
	public static final int RIGHT = -1;

	public static int WHICH_WHEEL = RIGHT;

	public static double X_MULTIPLIER = WHICH_WHEEL == RIGHT ? StandardTrackingWheelLocalizer.RIGHT_X_MULTIPLIER : StandardTrackingWheelLocalizer.LEFT_X_MULTIPLIER;
	//public static double X_MULTIPLIER = 71.0 / 71.493;
	public static double Y_MULTIPLIER = StandardTrackingWheelLocalizer.Y_MULTIPLIER;
	//public static double Y_MULTIPLIER = 71.0 / 71.339;

	public static double TICKS_PER_REV = StandardTrackingWheelLocalizer.TICKS_PER_REV;
	public static double WHEEL_RADIUS = StandardTrackingWheelLocalizer.WHEEL_RADIUS; // in
	public static double GEAR_RATIO = StandardTrackingWheelLocalizer.GEAR_RATIO; // output (wheel) speed / input (encoder) speed

	public static double PARALLEL_X = StandardTrackingWheelLocalizer.LATERAL_WHEEL_OFFSET; // X is the up and down direction
	public static double PARALLEL_Y = WHICH_WHEEL*StandardTrackingWheelLocalizer.LATERAL_DISTANCE / 2.0; // Y is the strafe direction

	public static double PERPENDICULAR_X = StandardTrackingWheelLocalizer.FORWARD_OFFSET;
	public static double PERPENDICULAR_Y = 0.0;

	// Parallel/Perpendicular to the forward axis
	// Parallel wheel is parallel to the forward axis
	// Perpendicular is perpendicular to the forward axis
	private Encoder parallelEncoder, perpendicularEncoder;

	private SampleMecanumDrive drive;

	public TwoWheelTrackingLocalizer(HardwareMap hardwareMap, SampleMecanumDrive drive) {
		super(Arrays.asList(
						new Pose2d(PARALLEL_X, PARALLEL_Y, 0),
						new Pose2d(PERPENDICULAR_X, PERPENDICULAR_Y, Math.toRadians(90))
		));

		this.drive = drive;

		parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, WHICH_WHEEL == LEFT ? "frontLeft" : "backRight"));
		perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "frontRight"));

		// reverse encoders
		if(WHICH_WHEEL == LEFT){
			parallelEncoder.setDirection(Encoder.Direction.REVERSE);
		}
		perpendicularEncoder.setDirection(Encoder.Direction.REVERSE);
	}

	public static double encoderTicksToInches(double ticks) {
		return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
	}

	@Override
	public double getHeading() {
		return drive.getRawExternalHeading();
	}

	@Override
	public Double getHeadingVelocity() {
		return drive.getExternalHeadingVelocity();
	}

	@NonNull
	@Override
	public List<Double> getWheelPositions() {
		return Arrays.asList(
						encoderTicksToInches(parallelEncoder.getCurrentPosition()) * X_MULTIPLIER,
						encoderTicksToInches(perpendicularEncoder.getCurrentPosition()) * Y_MULTIPLIER
		);
	}

	@NonNull
	@Override
	public List<Double> getWheelVelocities() {
		// TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
		//  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
		//  compensation method

		return Arrays.asList(
						encoderTicksToInches(parallelEncoder.getRawVelocity()) * X_MULTIPLIER,
						encoderTicksToInches(perpendicularEncoder.getRawVelocity()) * Y_MULTIPLIER
		);
	}
}
