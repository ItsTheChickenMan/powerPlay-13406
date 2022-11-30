package org.firstinspires.ftc.teamcode.lib.abe;

import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.motion.AngleAdjuster;
import org.firstinspires.ftc.teamcode.lib.motion.LinearSlides;
import org.firstinspires.ftc.teamcode.lib.motion.PositionableServo;
import org.firstinspires.ftc.teamcode.opmodes.AimAtPointTest;

/**
 * @brief The entire arm (angle adjuster, slides, wrist, and fingers) wrapped into one easy-to-use class
 */
public class AbeArm {
	private AngleAdjuster elbow;
	public LinearSlides slides;

	private PositionableServo wrist;
	private PositionableServo fingers;

	// overall... //
	private boolean manualControl; // is the programmer allowed to control the arm?

	// slides length for aiming at point
	private double aimSlidesLength;

	// elbow angle for aiming at point
	private double aimElbowAngle;

	// are we aiming rn?
	private boolean isAiming;

	// elbow... //

	// wrist... //
	private double wristAngle = 0; // relative to elbow, in degrees

	// fingers ... //
	private boolean fingersClamped;

	public AbeArm(AngleAdjuster elbow, LinearSlides slides, PositionableServo wrist, PositionableServo fingers){
		this.elbow = elbow;
		this.slides = slides;
		this.wrist = wrist;
		this.fingers = fingers;
	}

	// GETTERS & SETTERS //

	public void aimAt(double x, double y){
		this.aimAt(x, y, true);
	}

	/**
	 * @brief aim at a certain point, relative to where the robot currently is
	 *
	 * These coordinates are permanently relative to the bot's coordinates.  Basically, the bot moving won't update the arm on its own
	 *
	 * Overwrites any previous point present
	 *
	 * If the point can't be reached, the arm doesn't do anything
	 *
	 * @param x how far in front of the bot the point is
	 * @param y how far above/below the bot the point is
	 */
	public void aimAt(double x, double y, boolean accountForFalloff){
		// set slides extension
		// TODO: account for slide bending?
		this.aimSlidesLength = Math.sqrt(
						x*x + y*y - AbeConstants.ELBOW_RADIUS_INCHES*AbeConstants.ELBOW_RADIUS_INCHES
		);

		AimAtPointTest.globalTelemetry.addData("slides", this.aimSlidesLength);

		//double temp = Math.atan(y/x) - Math.atan(AbeConstants.ELBOW_RADIUS_INCHES / this.aimSlidesLength);

		// account for sagging (better word for this than "falloff" but too late now)
		/*if(accountForFalloff) {
			// determine height falloff counter
			// we use height for predicted falloff because it's easier to measure physically
			// this gets applied to y before it gets fed into the angle, and only the angle (slides length calculated normally)
			//double falloffPrediction = AbeConstants.getExpectedHeightFalloff(this.aimSlidesLength) * Math.cos(temp);

			//y += falloffPrediction;

			this.aimElbowAngle = Math.atan(y/x) - Math.atan(AbeConstants.ELBOW_RADIUS_INCHES / this.aimSlidesLength);
		} else {
			this.aimElbowAngle = temp;
		}*/

		// set elbow angle
		this.aimElbowAngle = Math.atan(y/x) - Math.atan(AbeConstants.ELBOW_RADIUS_INCHES / this.aimSlidesLength);

		// determine expected angular adjustment to account for falloff from stress
		if(accountForFalloff) {
			double stressFactor = Math.cos(this.aimElbowAngle)*this.aimSlidesLength - AbeConstants.ELBOW_TORQUE_FACTOR*AbeConstants.ELBOW_RADIUS_INCHES*Math.sin(this.aimElbowAngle);

			double falloffCounter = AbeConstants.getExpectedElbowSag(stressFactor);

			// adjust wrist for falloff counter
			this.positionWristRadians(-falloffCounter);

			this.aimElbowAngle += falloffCounter;
		}


		// we don't use this because I forgot that the LinearSlides class already accounts for the offset and just extends to the length it's told
		//this.aimSlidesLength -= AbeConstants.SLIDE_OFFSET_INCHES;

		//this.isAiming = false;
		this.isAiming = true;
	}

	/**
	 * @brief clears the robot's aim point, so it stops aiming
	 */
	public void clearAim(){
		this.isAiming = false;
	}

	/**
	 * @brief is the arm currently aiming at a point?
	 *
	 * @return true if it is, false if it isn't
	 */
	public boolean isAiming(){
		return this.isAiming;
	}

	/**
	 * @brief are the fingers currently clamped?
	 *
	 * @return true if they are, false if they're not
	 */
	public boolean getFingersClamped(){
		return this.fingersClamped;
	}

	/**
	 * @brief is manual programmer control enabled?
	 *
	 * @return true if it is, false if it isn't
	 */
	public boolean isManualControlEnabled(){
		return this.manualControl;
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
	 * @brief Adjust the wrist's angle for the elbow's angle
	 */
	public void updateWrist(){
		this.wrist.rotateToDegrees( -this.wrist.getMaxRangeDegrees()/2. + this.elbow.getAngleDegrees() + this.wristAngle);
	}

	/**
	 * @brief Update the entire arm (required in automatic mode)
	 */
	public void update(){
		// update slides length/elbow angle+
		if(this.isAiming() && !this.isManualControlEnabled()){
			// FIXME: allow velocity to be set by programmer
			this.elbow.rotateToRadians(this.aimElbowAngle, Math.PI/6.0);
			this.slides.extendTo(this.aimSlidesLength, 10.0);
		}

		// check elbow
		//this.elbow.check();

		// check slides
		this.slides.checkForBadExtension();

		// update wrist
		this.updateWrist();
	}
}
