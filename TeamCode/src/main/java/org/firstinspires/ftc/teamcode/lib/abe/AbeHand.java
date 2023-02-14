package org.firstinspires.ftc.teamcode.lib.abe;

import org.firstinspires.ftc.teamcode.lib.motion.PositionableServo;

/**
 * @brief Dedicated class for managing the flipper + wrist + claw servo combo
 */
public class AbeHand {
	/**
	 * @brief rolls the claw, allowing the cone to be flipped when the elbow flips over itself
	 */
	private PositionableServo flipperServo;

	/**
	 * @brief pitches the claw up and down, allowing the angle to stay consistent at all angle of the elbow and shoulder (and for the cone to be manipulated to guarantee deposits)
	 */
	private PositionableServo wristServo;

	/**
	 * @brief Clamps cones
	 */
	private PositionableServo clawServo;

	/**
	 * @brief Is the wrist flipped?  Based on calls to setRollToNormal/setRollToFlipped
	 */
	private boolean flipped;

	public AbeHand(PositionableServo flipperServo, PositionableServo wristServo){
		this.flipperServo = flipperServo;
		this.wristServo = wristServo;

		this.flipped = false;
	}

	/**
	 * @brief Set the pitch of the wrist (actuates wrist servo) in radians
	 *
	 * @param pitch
	 */
	public void setPitchRadians(double pitch){
		this.wristServo.rotateToRadians(pitch);
	}

	/**
	 * @brief Set the pitch of the wrist (actuates wrist servo) in degrees
	 *
	 * @param pitch
	 */
	public void setPitchDegrees(double pitch){
		this.wristServo.rotateToDegrees(pitch);
	}

	/**
	 * @brief Set the roll of the wrist (actuates the flipper servo) to upright
	 */
	public void setRollToNormal(){
		this.flipperServo.rotateToDegrees(0.0);

		this.flipped = false;
	}

	/**
	 * @brief Set the roll of the wrist (actuates the flipper servo) to flipped
	 */
	public void setRollToFlipped(){
		this.flipperServo.rotateToDegrees(180.0);

		this.flipped = true;
	}

	public void clamp(){
		this.clawServo.rotateToDegrees(AbeConstants.CLAW_CLOSED_ANGLE_DEGREES);
	}

	public void unclamp(){
		this.clawServo.rotateToDegrees(AbeConstants.CLAW_OPEN_ANGLE_DEGREES);
	}
}