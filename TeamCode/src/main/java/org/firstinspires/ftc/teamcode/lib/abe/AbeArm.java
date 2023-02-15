package org.firstinspires.ftc.teamcode.lib.abe;

import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.lib.motion.LinearSlides;
import org.firstinspires.ftc.teamcode.lib.motion.PositionableMotor;

/**
 * @brief Class for managing the abe arm system (shoulder, elbow, slides, wrist, claw)
 */
public class AbeArm {
	// hardware... //
	private PositionableMotor shoulderMotor;
	private PositionableMotor elbowMotor;

	private LinearSlides slides;

	private AbeHand hand;

	// aim details... //
	private double shoulderAngle;
	private double elbowAngle;
	private double slidesLength;

	private double desiredWristAngle;
	private double actualWristAngle;

	public AbeArm(PositionableMotor shoulderMotor, PositionableMotor elbowMotor, LinearSlides slides, AbeHand hand){
		this.shoulderMotor = shoulderMotor;
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
	 */
	public void setDesiredWristAngleDegrees(double angle){
		this.setDesiredWristAngleRadians(Math.toRadians(angle));
	}

	/**
	 * @brief set the hand upright
	 *
	 * this is uncontrolled by the aiming, must be managed outside
	 */
	public void setHandUpright(){
		this.hand.setRollToNormal();
	}

	/**
	 * @brief set the hand flipped
	 *
	 * this is uncontrolled by the aiming, must be managed outside
	 */
	public void setHandFlipped(){
		this.hand.setRollToFlipped();
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
	 * @brief Fetch the coordinates that the shoulder and slides will aim at from (x, y) arm space coordinates and an arbitrary elbow angle
	 *
	 * Doesn't consider max length of slides or elbow limits, so any elbow angle is valid
	 *
	 * @param x x coordinate in arm space
	 * @param y y coordinate in arm space
	 * @param a arbitrary elbow angle
	 * @return aim coordinates for shoulder as RR vector
	 */
	private Vector2d getAimCoordsFromElbowAngle(double x, double y, double a){
		return new Vector2d(x - AbeConstants.ELBOW_LENGTH_INCHES * Math.cos(a), y - AbeConstants.ELBOW_LENGTH_INCHES * Math.sin(a));
	}

	/**
	 * @brief get the actual elbow angle (measured from elbow) from the theoretical angle (measured from imaginary flat line)
	 *
	 * @param a
	 * @return
	 */
	private double getActualElbowAngle(double a){
		return this.shoulderAngle - a;
	}

	/**
	 * @brief get the theoretical elbow angle (measured from imaginary flat line) from the actual angle (measured from elbow)
	 *
	 * @param a
	 * @return
	 */
	private double getTheoreticalElbowAngle(double a){
		return this.shoulderAngle + a;
	}
	/**
	 * @brief Calculate the slides length required to aim at a point (x, y) in arm space with an arbitrary elbow angle of a
	 *
	 * Doesn't consider max length of slides or elbow limits, so any elbow angle is valid
	 *
	 * @param x x coordinate in arm space
	 * @param y y coordinate in arm space
	 * @param a arbitrary elbow angle
	 */
	public double calculateSlidesLength(double x, double y, double a){
		// calculate (x1, y1)
		Vector2d aimCoords = this.getAimCoordsFromElbowAngle(x, y, a);
		double x1 = aimCoords.getX();
		double y1 = aimCoords.getY();

		// calculate slides length
		double r = AbeConstants.SHOULDER_RADIUS_INCHES;
		double slidesLength = Math.sqrt(x1*x1 + y1*y1 - r*r);

		return slidesLength;
	}

	/**
	 * @brief Calculate the shoulder angle required to aim at a point (x, y) in arm space with an arbitrary elbow angle of a
	 *
	 * Doesn't consider max length of slides or elbow limits, so any elbow angle is valid
	 *
	 * @param x x coordinate in arm space
	 * @param y y coordinate in arm space
	 * @param a arbitrary elbow angle
	 */
	public double calculateShoulderAngle(double x, double y, double a){
		// calculate (x1, y1)
		Vector2d aimCoords = this.getAimCoordsFromElbowAngle(x, y, a);
		double x1 = aimCoords.getX();
		double y1 = aimCoords.getY();

		// calculate shoulder angle
		double r = AbeConstants.SHOULDER_RADIUS_INCHES;
		double shoulderAngle = Math.atan(y1/x1) - Math.asin(r / aimCoords.norm());

		return shoulderAngle;
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
		// calculate arbitrary elbow angle
		double r = AbeConstants.SHOULDER_RADIUS_INCHES;
		double elbowAngle = Math.atan(y/x) - Math.asin(r / Math.sqrt(x*x + y*y));

		// calculate shoulder angle + slides length
		double shoulderAngle = this.calculateShoulderAngle(x, y, elbowAngle);
		double slidesLength = this.calculateSlidesLength(x, y, elbowAngle);

		// calculate actual elbow angle
		double actualElbowAngle = this.getActualElbowAngle(elbowAngle);

		// calculate wrist angle
		this.actualWristAngle = this.desiredWristAngle - elbowAngle;

		// save values
		this.shoulderAngle = shoulderAngle;
		this.slidesLength = slidesLength;
		this.elbowAngle = actualElbowAngle;
	}

	/**
	 * @brief Update the arm, moving all parts by default
	 */
	public void update(){
		this.update(true, true, true);
	}

	/**
	 * @brief Update the arm and choose which parts are allowed to act as a result of aim changes
	 *
	 * @param doShoulder
	 * @param doSlides
	 * @param doElbow
	 */
	public void update(boolean doShoulder, boolean doSlides, boolean doElbow){
		// aim shoulder, if told to
		if(doShoulder){
			this.shoulderMotor.rotateAngleRadians(this.shoulderAngle, AbeConstants.SHOULDER_VELOCITY_RADIANS);
		}

		// aim slides, if told to
		if(doSlides){
			this.slides.extendTo(this.slidesLength, AbeConstants.SLIDES_VELOCITY_INCHES);
		}

		// aim elbow, if told to
		if(doElbow){
			this.elbowMotor.rotateAngleRadians(this.elbowAngle, AbeConstants.ELBOW_VELOCITY_RADIANS);
		}

		// do hand
		this.hand.setPitchRadians(this.actualWristAngle);
	}
}
