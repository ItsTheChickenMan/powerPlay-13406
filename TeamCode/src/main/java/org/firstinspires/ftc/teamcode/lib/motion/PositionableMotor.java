package org.firstinspires.ftc.teamcode.lib.motion;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class PositionableMotor {
	// motor
	private DcMotorEx motor;
	private int tickOffset = 0; // current tick offset from the number returned by getCurrentPosition, added to whenever resetEncoders is called

	// details
	private double gearRatio; // NOTE: ratio of driver gear rotations : driven gear rotations
	private double tickRatio;

	// disabled
	private boolean disabled;

	public PositionableMotor(DcMotorEx motor, double gearRatio, double tickRatio){
		this.motor = motor;
		this.gearRatio = gearRatio;
		this.tickRatio = tickRatio;

		// brake at zero
		this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		// reset encoders
		this.resetEncoders();

		// enable
		this.enable();
	}

	/**
	 * @brief abstraction for is busy
	 *
	 * @return true if motor is busy
	 */
	public boolean isBusy(){
		return this.motor.isBusy();
	}

	/**
	 * @brief Reset the motor encoders
	 */
	public void resetEncoders(){
		this.tickOffset += this.motor.getCurrentPosition();
		this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
	}

	public void disable(){
		// stops the motor from running to position
		this.resetEncoders();

		// might stop it quicker?
		this.motor.setPower(0);

		// prevents any other calls from moving the motor, provided that the user doesn't use it directly
		this.disabled = true;
	}

	public void enable(){
		this.disabled = false;
	}

	/**
	 * @brief gets the cumulative number of rotations of the driven gear (even if encoders are reset) from start
	 *
	 * @return number of rotations
	 */
	public double getRotations() {
		// get current rotations from ticks + gear ratio
		int position = this.motor.getCurrentPosition();
		double rotations = (double)(position + this.tickOffset) / this.tickRatio;

		double drivenRotations = rotations / this.gearRatio;

		return drivenRotations;
	}

	/**
	 * @brief gets the cumulative angle (even if encoders are reset) from start
	 *
	 * @return angle in radians
	 */
	public double getAngleRadians() {
		// get current rotations * radians per rotation
		return this.getRotations() * Math.PI*2;
	}

	/**
	 * @brief gets the cumulative angle (even if encoders are reset) from start
	 *
	 * @return angle in degrees
	 */
	public double getAngleDegrees(){
		// get current rotations * degrees per rotation
		return this.getRotations() * 360.0;
	}

	/**
	 * @brief rotate the driven gear a certain amount of rotations
	 *
	 * @param rotations desired amount of rotations
	 * @param velocity in rotations
	 */
	public void rotate(double rotations, double velocity){
		if(this.disabled) return;

		// position calculation
		double drivingRotations = rotations * this.gearRatio;
		double ticks = drivingRotations * this.tickRatio;

		// velocity calculation
		double drivingVelocity = velocity * this.gearRatio;
		double velocityTicks = drivingVelocity * this.tickRatio;

		this.resetEncoders();

		this.motor.setTargetPosition((int)ticks);

		this.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

		this.motor.setVelocity(velocityTicks);
	}

	/**
	 * @brief rotate the motor a certain amount of degrees
	 *
	 * @param angle angle in degrees
	 * @param velocity velocity in degrees
	 */
	public void rotateAngleDegrees(double angle, double velocity){
		double rotations = angle / 360.0;
		double velocityRotations = velocity / 360.0;

		this.rotate(rotations, velocityRotations);
	}

	/**
	 * @brief rotate the motor a certain amount of radians
	 *
	 * @param angle angle in radians
	 * @param velocity velocity in radians
	 */
	public void rotateAngleRadians(double angle, double velocity){
		double rotations = angle / (2*Math.PI);
		double velocityRotations = velocity / (2*Math.PI);

		this.rotate(rotations, velocityRotations);
	}

	/**
	 * @brief rotate the motor a certain number of rotations per second (not going to any particular position)
	 *
	 * This preserves the position of the motor despite resetting encoders
	 *
	 * @param rotationsPerSecond
	 */
	public void rotateSpeed(double rotationsPerSecond){
		// velocity calculation
		double drivingVelocity = rotationsPerSecond * this.gearRatio;
		double drivingTicks = drivingVelocity * this.tickRatio;

		this.resetEncoders();

		this.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

		this.motor.setVelocity(drivingTicks);
	}

	/**
	 * @brief rotate the motor at a certain speed defined by degrees
	 *
	 * @param angularVelocity
	 */
	public void rotateSpeedDegrees(double angularVelocity){
		double rotationsPerSecond = angularVelocity / 360.0;

		this.rotateSpeed(rotationsPerSecond);
	}

	public void rotateSpeedRadians(double angularVelocity){
		double rotationsPerSecond = angularVelocity / (2*Math.PI);

		this.rotateSpeed(rotationsPerSecond);
	}
}
