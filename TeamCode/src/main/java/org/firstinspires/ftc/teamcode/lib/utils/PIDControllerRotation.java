package org.firstinspires.ftc.teamcode.lib.utils;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * @brief PIDController specifically for rotation (just uses a special error method, nothing else)
 */
public class PIDControllerRotation extends PIDController {
	public PIDControllerRotation(double p, double i, double d){
		super(p, i, d);
	}

	public PIDControllerRotation(double p, double i, double d, ElapsedTime timer){
		super(p, i, d, timer);
	}

	@Override
	public double getError(){
		return AngleHelper.angularDistanceRadians(this.target, this.measured);
	}
}
