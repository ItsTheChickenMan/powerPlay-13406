package org.firstinspires.ftc.teamcode.lib.utils;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * @brief PIDController specifically for rotation
 */
public class PIDControllerRotation extends PIDController {
	public enum RotationDirection {
		FASTEST,
		CCW,
		CW
	}

	private RotationDirection direction = RotationDirection.FASTEST;

	private double tolerance = Math.toRadians(35);

	public PIDControllerRotation(double p, double i, double d){
		super(p, i, d);
	}

	public PIDControllerRotation(PIDCoefficients coefficients){
		super(coefficients);
	}

	public PIDControllerRotation(double p, double i, double d, ElapsedTime timer){
		super(p, i, d, timer);
	}

	public PIDControllerRotation(PIDCoefficients coefficients, ElapsedTime timer){
		super(coefficients, timer);
	}

	public void setDirection(RotationDirection direction){
		this.direction = direction;
	}

	public void setToleranceDegrees(double tolerance){
		this.tolerance = Math.toRadians(tolerance);
	}

	public void setToleranceRadians(double tolerance){
		this.tolerance = tolerance;
	}

	@Override
	public double getLastError(){
		double distance = AngleHelper.angularDistanceRadians(this.measured, this.target);

		// flip distance to one direction if necessary
		if(Math.abs(distance) > this.tolerance){
			if(this.direction == RotationDirection.CCW && distance < 0){
				distance = Math.PI*2 + distance;
			} else if(this.direction == RotationDirection.CW && distance > 0){
				distance = distance - Math.PI*2;
			}
		}

		return distance;
	}
}
