package org.firstinspires.ftc.teamcode.lib.utils;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * @brief PIDController specifically for rotation (just uses a special error method, nothing else)
 */
public class PIDControllerRotation extends PIDController {
	public static int FASTEST = 0;
	public static int CCW = -1;
	public static int CW = 1;

	private int direction = FASTEST;

	public PIDControllerRotation(double p, double i, double d){
		super(p, i, d);
	}

	public PIDControllerRotation(double p, double i, double d, ElapsedTime timer){
		super(p, i, d, timer);
	}

	public void setDirection(int direction){
		// discard invalid
		if(direction + 1 > 2) return;

		this.direction = direction;
	}

	@Override
	public double getError(){
		double distance = AngleHelper.angularDistanceRadians(this.target, this.measured);

		if(distance > Math.toRadians(15) && ((this.direction == CCW && distance > 0) || (this.direction == CW && distance < 0))){
			distance = Math.toRadians(360) - distance;
		}

		return distance;
	}
}
