package org.firstinspires.ftc.teamcode.lib.abe;

import org.firstinspires.ftc.teamcode.lib.motion.LinearSlides;
import org.firstinspires.ftc.teamcode.lib.motion.PositionableMotor;

/**
 * @brief Class for managing the abe arm system (elbow, slides, wrist, claw)
 */
public class AbeArm {
	// hardware... //
	private PositionableMotor elbowMotor;

	private LinearSlides slides;

	private AbeHand hand;

	// aim details... //
	private double elbowAngle;
	private double slidesLength;

	private double desiredWristAngle;
	private double actualWristAngle;

	public AbeArm(PositionableMotor elbowMotor, LinearSlides slides, AbeHand hand){
		this.slides = slides;
		this.elbowMotor = elbowMotor;
		this.hand = hand;
	}

	/**
	 * @brief get the currently desired wrist rotation in radians
	 */
	public double getDesiredWristAngleRadians(){
		return this.desiredWristAngle;
	}

	/**
	 * @brief get the currently desired wrist rotation in degrees
	 *
	 * this is uncontrolled by the aiming, must be managed outside
	 */
	public double getDesiredWristAngleDegrees(){
		return Math.toDegrees(this.getDesiredWristAngleRadians());
	}

	/**
	 * @brief set the currently desired wrist rotation in radians
	 */
	public void setDesiredWristAngleRadians(double angle){
		this.desiredWristAngle = angle;
	}

	/**
	 * @brief set the currently desired wrist rotation in degrees
	 *
	 * this is uncontrolled by the aiming, must be managed outside
	 */
	public void setDesiredWristAngleDegrees(double angle){
		this.setDesiredWristAngleRadians(Math.toRadians(angle));
	}

	/**
	 * @brief set the hand to clamped
	 *
	 * this is uncontrolled by the aiming, must be managed outside
	 */
	public void setHandClamped(){
		this.hand.clamp();
	}

	/**
	 * @brief set the hand to unclamped
	 *
	 * this is uncontrolled by the aiming, must be managed outside
	 */
	public void setHandUnclamped(){
		this.hand.unclamp();
	}

	/**
	 * @brief manually set the elbow angle to some value in radians
	 */
	public void setElbowAngleRadians(double elbowAngle){
		this.elbowAngle = elbowAngle;
	}

	/**
	 * @brief manually set the elbow angle to some value in degrees
	 */
	public void setElbowAngleDegrees(double elbowAngle){
		this.setElbowAngleRadians(Math.toRadians(elbowAngle));
	}

	/**
	 * @brief manually set slides length to some value in inches
	 */
	public void setSlidesLengthInches(double slidesLength){
		this.slidesLength = slidesLength;
	}

	/**
	 * @brief Calculate the slides length required to aim at a point (x, y) in arm space
	 *
	 * @param x x coordinate in arm space
	 * @param y y coordinate in arm space
	 */
	public double calculateSlidesLength(double x, double y){
		// calculate slides length
		double r = AbeConstants.ELBOW_RADIUS_INCHES;
		double slidesLength = Math.sqrt(x*x + y*y - r*r);

		return slidesLength;
	}

	/**
	 * @brief Calculate the elbow angle required to aim at a point (x, y) in arm space
	 *
	 * @param x x coordinate in arm space
	 * @param y y coordinate in arm space
	 */
	public double calculateElbowAngle(double x, double y){
		// calculate elbow angle
		double r = AbeConstants.ELBOW_RADIUS_INCHES;
		double distance = Math.sqrt(x*x + y*y);
		double elbowAngle = Math.atan(y/x) - Math.asin(r / distance);

		return elbowAngle;
	}

	/**
	 * @brief Instructs the arm to aim at a point in its idea of space, that being a two dimensional plane in which x is horizontal distance from the arm's point of rotation and y is vertical distance from the arm's point of rotation
	 *
	 * Requires at least one call to update
	 *
	 * This calculates the "arbitrary elbow angle" itself based on what requires the least slides movement
	 *
	 * @param x x coordinate in arm space
	 * @param y y coordinate in arm space
	 */
	public void aimAt(double x, double y){
		// calculate elbow angle + slides length
		double elbowAngle = this.calculateElbowAngle(x, y);
		double slidesLength = this.calculateSlidesLength(x, y);

		// calculate wrist angle
		this.actualWristAngle = this.desiredWristAngle - elbowAngle;

		// save values
		this.elbowAngle = elbowAngle;
		this.slidesLength = slidesLength;
	}

	/**
	 * @brief Update the arm, moving all parts by default
	 */
	public void update(){
		this.update(true, true);
	}

	/**
	 * @brief Update the arm and choose which parts are allowed to act as a result of aim changes
	 *
	 * @param doSlides
	 * @param doElbow
	 */
	public void update(boolean doElbow, boolean doSlides){
		// aim elbow, if told to
		if(doElbow){
			this.elbowMotor.rotateAngleRadians(this.elbowAngle, AbeConstants.ELBOW_VELOCITY_RADIANS);
		}

		// aim slides, if told to
		if(doSlides){
			this.slides.extendTo(this.slidesLength, AbeConstants.SLIDES_VELOCITY_INCHES);
		}

		// do hand
		this.hand.setPitchRadians(this.actualWristAngle);
	}
}
