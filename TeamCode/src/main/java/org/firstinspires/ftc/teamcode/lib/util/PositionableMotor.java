package org.firstinspires.ftc.teamcode.lib.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class PositionableMotor {
	// motor
	private DcMotorEx motor;

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
	}

	/**
	 * @brief Reset the motor encoders
	 */
	public void resetEncoders(){
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
	 * @brief rotate the driven gear a certain amount of *rotations*
	 *
	 * @param rotations desired amount of *rotations*
	 * @param velocity in *rotations*
	 */
	public void rotate(double rotations, double velocity){
		if(this.disabled) return;

		// position calculation
		double drivingRotations = rotations * this.gearRatio;
		double ticks = drivingRotations * this.tickRatio;

		// velocity calculation
		double drivingVelocity = velocity * this.gearRatio;
		double velocityTicks = drivingVelocity * this.tickRatio;

		this.motor.setTargetPosition((int)ticks);

		this.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

		this.motor.setVelocity(velocityTicks);
	}

	/**
	 * @brief rotate the motor a certain amount of *degrees*
	 *
	 * @param angle angle in *degrees*
	 * @param velocity velocity in *degrees*
	 */
	public void rotateAngle(double angle, double velocity){
		double rotations = angle / 360.0;
		double velocityRotations  = velocity / 360.0;

		this.rotate(rotations, velocityRotations);
	}
}
