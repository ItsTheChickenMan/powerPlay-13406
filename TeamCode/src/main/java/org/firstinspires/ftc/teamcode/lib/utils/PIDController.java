package org.firstinspires.ftc.teamcode.lib.utils;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.analysis.integration.IterativeLegendreGaussIntegrator;

/**
 * @brief super basic and quick PID controller implementation
 */
public class PIDController {
	private double p_const;
	private double i_const;
	private double d_const;

	private ElapsedTime timer;

	private double delta;
	private double lastTime;

	private double lastError;

	private double accumulativeError;

	protected double target;
	protected double measured;

	public PIDController(double p, double i, double d){
		this(p, i, d, new ElapsedTime());
	}

	public PIDController(PIDCoefficients coefficients){
		this(coefficients.kP, coefficients.kI, coefficients.kD);
	}

	public PIDController(double p, double i, double d, ElapsedTime timer){
		this.p_const = p;
		this.i_const = i;
		this.d_const = d;

		this.timer = timer;

		this.lastTime = this.timer.seconds();
	}

	public PIDController(PIDCoefficients coefficients, ElapsedTime timer){
		this(coefficients.kP, coefficients.kI, coefficients.kD, timer);
	}

	public void setTargetPosition(double target){
		this.target = target;
	}

	public double getLastError(){
		return this.target - this.measured;
	}

	public double getLastDerivative(){
			return (this.getLastError() - this.lastError) / this.delta;
	}

	public double getIntegral(){
		return this.accumulativeError;
	}

	public double update(double measured){
		// get time since last call
		double time = this.timer.seconds();
		this.delta = time - this.lastTime;
		this.lastTime = time;

		// update measured
		this.measured = measured;

		double error = this.getLastError();

		if(Double.isNaN(error)) return 0.0;

		double p = error;

		double d = (error - this.lastError) / this.delta;

		this.lastError = error;

		this.accumulativeError += error * this.delta;

		double i = error * this.accumulativeError;

		return this.p_const*p + this.i_const*i + this.d_const*d;
	}
}