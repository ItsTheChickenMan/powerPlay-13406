package org.firstinspires.ftc.teamcode.lib.utils;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * @brief PIDController specifically for rotation (just uses a special error method, nothing else)
 */
public class PIDControllerRotation extends PIDController {
	public enum RotationDirection {
		FASTEST,
		CCW,
		CW
	}

	private RotationDirection direction = RotationDirection.FASTEST;

	private double tolerance = Math.toRadians(15.0);

	public PIDControllerRotation(double p, double i, double d){
		super(p, i, d);
	}

	public PIDControllerRotation(double p, double i, double d, ElapsedTime timer){
		super(p, i, d, timer);
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
	public double getError(){
		double distance = AngleHelper.angularDistanceRadians(this.target, this.measured);

		// flip distance to one direction if necessary
		if(Math.abs(distance) > this.tolerance && ((this.direction == RotationDirection.CCW && distance > 0) || (this.direction == RotationDirection.CW && distance < 0))){
			distance = Math.PI*2 - distance;
		}

		return distance;
	}
}
