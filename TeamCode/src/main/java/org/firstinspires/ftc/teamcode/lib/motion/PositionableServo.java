package org.firstinspires.ftc.teamcode.lib.motion;


import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.lib.abe.AbeConstants;

public class PositionableServo {
	private Servo servo;
	private double zeroAngleRadians = 0.0;
	private double maxRange = Math.toRadians(300); // max range (defaults to 300 degrees, which is gobilda default)

	/**
	 * @brief Initialize a new positionable servo
	 *
	 * Less accurate than a motor, but has the advantage of being easier to manage than a motor because each position always corresponds to the same angle
	 *
	 * @param servo the servo to use
	 */
	public PositionableServo(Servo servo) {
		this.servo = servo;
	}

	/**
	 * @brief Initialize a new positionable servo with a custom max range
	 *
	 * @param servo
	 * @param maxRange
	 */
	public PositionableServo(Servo servo, double maxRange){
		this.servo = servo;
		this.maxRange = maxRange;
		this.zeroAngleRadians = 0.0;
	}

	/**
	 * @brief Initialize a new positionable servo with a custom max range and zero angle (angle to use as 0 instead of the default)
	 *
	 * @param servo
	 * @param maxRange
	 * @param zeroAngleRadians angle to use as 0 degrees in place of the default (0 degrees).  ex. if this is 35 degrees, telling the servo to go to 0 degrees will actually make it go to 35 degrees, telling it to go to 10 degrees will actually make it go to 45 degrees, etc.
	 */
	public PositionableServo(Servo servo, double maxRange, double zeroAngleRadians){
		this.servo = servo;
		this.maxRange = maxRange;
		this.zeroAngleRadians = zeroAngleRadians;
	}

	public double getZeroAngleRadians(){
		return this.zeroAngleRadians;
	}

	public void setZeroAngleRadians(double angle){
		this.zeroAngleRadians = angle;
	}

	public double getMaxRangeRadians(){
		return this.maxRange;
	}

	public double getMaxRangeDegrees(){
		return Math.toDegrees(this.maxRange);
	}

	public double getAngleRadians(){
		double position = this.servo.getPosition();

		double angle = position * this.maxRange;

		angle -= this.maxRange / 2.;

		angle += this.zeroAngleRadians;

		return angle;
	}

	public double getAngleDegrees(){
		return Math.toDegrees(this.getAngleRadians());
	}

	/**
	 * @brief Rotate to a certain angle, in radians (between -range/2 and range/2)
	 *
	 * @param angle angle in radians to rotate to
	 */
	public void rotateToRadians(double angle){
		// subtract offset
		angle -= this.zeroAngleRadians;

		// clamp angle
		angle = Range.clip(angle, -this.maxRange/2., this.maxRange/2.);

		angle += this.maxRange/2.;

		// translate angle to position via linear interpolation
		double position = angle / this.maxRange;

		// set position
		this.servo.setPosition(position);
	}

	/**
	 * @brief Rotate to a certain angle, in degrees (between -range/2 and range/2)
	 *
	 * @param degrees angle in degrees to rotate to
	 */
	public void rotateToDegrees(double degrees){
		this.rotateToRadians(Math.toRadians(degrees));
	}
}