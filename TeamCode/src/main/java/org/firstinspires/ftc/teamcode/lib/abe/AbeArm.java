package org.firstinspires.ftc.teamcode.lib.abe;

import org.firstinspires.ftc.teamcode.lib.motion.PositionableMotor;

/**
 * @brief Class for managing the abe arm system (shoulder, elbow, slides, wrist, claw)
 */
public class AbeArm {
	// hardware... //
	private PositionableMotor shoulderMotor;
	private PositionableMotor elbowMotor;
	private PositionableMotor slidesMotor;

	private AbeHand hand;

	// aim details... //
	private double shoulderAngle;
	private double elbowAngle;
	private double wristAngle;

	public AbeArm(PositionableMotor shoulderMotor, PositionableMotor slidesMotor, PositionableMotor elbowMotor, AbeHand hand){
		this.shoulderMotor = shoulderMotor;
		this.slidesMotor = slidesMotor;
		this.elbowMotor = elbowMotor;
		this.hand = hand;
	}

	public void update(){

	}
}
