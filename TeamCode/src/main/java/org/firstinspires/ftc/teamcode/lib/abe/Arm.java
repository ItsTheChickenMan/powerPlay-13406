package org.firstinspires.ftc.teamcode.lib.abe;

import org.firstinspires.ftc.teamcode.lib.motion.AngleAdjuster;
import org.firstinspires.ftc.teamcode.lib.motion.LinearSlides;
import org.firstinspires.ftc.teamcode.lib.motion.PositionableServo;

/**
 * @brief The entire arm (angle adjuster, slides, wrist, and fingers) wrapped into one easy-to-use class
 */
public class Arm {
	private AngleAdjuster elbow;
	public LinearSlides slides;

	private PositionableServo wrist;
	private PositionableServo fingers;

	// overall... //
	private boolean manualControl; // is the programmer allowed to control the arm?

	// elbow... //

	// wrist... //
	private double wristAngle = 0; // relative to elbow, in degrees

	// fingers ... //
	private boolean fingersClamped;

	public Arm(AngleAdjuster elbow, LinearSlides slides, PositionableServo wrist, PositionableServo fingers){
		this.elbow = elbow;
		this.slides = slides;
		this.wrist = wrist;
		this.fingers = fingers;
	}

	// GETTERS & SETTERS //

	/**
	 * @brief are the fingers currently clamped?
	 *
	 * @return true if they are, false if they're not
	 */
	public boolean getFingersClamped(){
		return this.fingersClamped;
	}

	/**
	 * @brief Allow the programmer to use any method that gives them direct control of a component.
	 */
	public void enableManualControl(){
		this.manualControl = true;
	}

	/**
	 * @brief Disallow the programmer from using any method that gives them direct control of a component.
	 */
	public void disableManualControl(){
		this.manualControl = false;
	}

	/**
	 * @brief set manual control to some value
	 *
	 * @param enabled true if enabling manual control, false if disabling manual control
	 */
	public void setManualControl(boolean enabled){
		this.manualControl = enabled;
	}

	public double getElbowAngleDegrees(){
		return this.elbow.getAngleDegrees();
	}

	public double getElbowAngleRadians(){
		return this.elbow.getAngleRadians();
	}

	// MANUAL CONTROL (methods that silently fail if this.manualControl is false) //

	/**
	 * @brief move elbow to a position if manual control is enabled
	 *
	 * @param degrees
	 * @param velocity degrees / second
	 */
	public void setElbowAngleDegrees(double degrees, double velocity){
		if(!this.manualControl) return;

		this.elbow.rotateToDegrees(degrees, velocity);
	}

	/**
	 * @brief move elbow to a position if manual control is enabled
	 *
	 * @param radians
	 * @param velocity radians / second
	 */
	public void setElbowAngleRadians(double radians, double velocity){
		if(!this.manualControl) return;

		this.elbow.rotateToRadians(radians, velocity);
	}

	/**
	 * @brief rotate elbow at a certain speed if manual control is enabled
	 *
	 * @param velocity degrees / second
	 */
	public void setElbowSpeedDegrees(double velocity){
		if(!this.manualControl) return;

		this.elbow.rotateSpeedDegrees(velocity);
	}

	/**
	 * @brief rotate elbow at a certain speed if manual control is enabled
	 *
	 * @param velocity radians / second
	 */
	public void setElbowSpeedRadians(double velocity) {
		if (!this.manualControl) return;

		this.elbow.rotateSpeedRadians(velocity);
	}

	/**
	 * @brief Extend slides to a certain position, if manual control is enabled
	 *
	 * @param extension position to extend to (see LinearSlides docs)
	 * @param velocity units / second (see LinearSlides docs)
	 */
	public void extendSlidesTo(double extension, double velocity){
		if(!this.manualControl) return;

		this.slides.extendTo(extension, velocity);
	}

	/**
	 * @brief Extend slides a certain amount, if manual control is enabled
	 *
	 * @param extension amount to extend (see LinearSlides docs)
	 * @param velocity units / second (see LinearSlides docs)
	 */
	public void extendSlides(double extension, double velocity){
		if(!this.manualControl) return;

		this.slides.extend(extension, velocity);
	}

	public void freezeSlides(){
		if(!this.manualControl) return;

		this.slides.freeze();
	}

	public void unfreezeSlides(){
		if(!this.manualControl) return;

		this.slides.unfreeze();
	}

	/**
	 * @brief set the slides to go a particular speed if manual control is enabled
	 *
	 * @param velocity inches/second
	 */
	public void setSlidesSpeed(double velocity){
		if(!this.manualControl) return;

		this.slides.setVelocity(velocity);
	}

	// AUTOMATIC CONTROL //

	/**
	 * @brief Rotate the wrist to a certain orientation, relative to the elbow's orientation
	 *
	 * @param angle angle, in degrees
	 */
	public void positionWristDegrees(double angle){
		this.wristAngle = angle;
	}

	/**
	 * @brief Rotate the wrist to a certain orientation, relative to the elbow's orientation
	 *
	 * @param angle, in radians
	 */
	public void positionWristRadians(double angle){
		this.wristAngle = Math.toDegrees(angle);
	}

	/**
	 * @brief clamp the fingers, regardless of whether they are currently clamped or not
	 */
	public void clampFingers(){
		this.fingersClamped = true;

		this.fingers.rotateToDegrees( AbeConstants.FINGERS_CLOSED_POSITION );
	}

	/**
	 * @brief unclamp the fingers, regardless of whether they are currently unclamped or not
	 */
	public void unclampFingers(){
		this.fingersClamped = false;

		this.fingers.rotateToDegrees( AbeConstants.FINGERS_OPEN_POSITION );
	}

	/**
	 * @brief toggle whether the fingers are clamped or unclamped
	 */
	public void toggleFingers(){
		this.fingersClamped = !this.fingersClamped;

		// TODO: funky logic here (fingersClamped is used to check, and also to set?)
		if(this.fingersClamped){
			this.clampFingers();
		} else {
			this.unclampFingers();
		}
	}

	/**
	 * @brief Bring the end of the arm to a certain point in front of it from the point where the elbow rotates
	 *
	 * @param x forward/backward from robot
	 * @param y up/down from robot
	 */
	public void goToPoint(double x, double y, double elbowVelocity, double slidesVelocity){
		// angle of angle adjuster is atan2(y/x)
		double angle = Math.atan2(y, x);

		// length of the slides (from elbow's point of rotation)
		double length = Math.sqrt(x*x + y*y);

		// using multithreading to angle elbow and then extend
		Runnable movementRunner = () -> {
			this.elbow.rotateAngleRadians(angle, elbowVelocity);

			// FIXME: problem with opModeIsActive()?
			while(this.elbow.isBusy());

			this.slides.extendTo(length, slidesVelocity);
		};

		Thread movementThread = new Thread(movementRunner);

		movementThread.start();
	}

	/**
	 * @brief Adjust the wrist's angle for the elbow's angle
	 */
	public void updateWrist(){
		this.wrist.rotateToDegrees( -this.wrist.getMaxRangeDegrees()/2. - this.elbow.getAngleDegrees() + this.wristAngle);
	}

	/**
	 * @brief Update the entire arm (required in automatic mode)
	 */
	public void update(){
		// check elbow
		this.elbow.check();

		// check slides
		// TODO: have this set by programmer
		//this.slides.checkForBadExtension();

		// update wrist
		this.updateWrist();
	}
}
