package org.firstinspires.ftc.teamcode.lib.abe;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.lib.motion.AngleAdjuster;
import org.firstinspires.ftc.teamcode.lib.motion.LinearSlidesEx;
import org.firstinspires.ftc.teamcode.lib.motion.PositionableServo;
import org.firstinspires.ftc.teamcode.lib.utils.GlobalStorage;

/**
 * @brief The entire arm (angle adjuster, slides, wrist, and fingers) wrapped into one easy-to-use class
 */
public class AbeArm {
	// constants //
	public static final double ELBOW_STEADY_STATE_ERROR_TOLERANCE = Math.toRadians(1.0);
	public static final double ELBOW_STEADY_STATE_DERIVATIVE_TOLERANCE = Math.toRadians(0.2);

	public static final double SLIDES_STEADY_STATE_ERROR_TOLERANCE = 0.75;
	public static final double SLIDES_STEADY_STATE_DERIVATIVE_TOLERANCE = 0.5;

	// members //

	private AngleAdjuster elbow;
	public LinearSlidesEx slides;

	private PositionableServo wrist;
	private PositionableServo fingers;

	// overall... //

	// FIXME: too many timers in class instances?  might be better to have a global timer passed to constructor (at least as an option)
	private ElapsedTime timer;
	private double lastTime;

	private boolean manualControl; // is the programmer allowed to control the arm?

	// are we aiming rn?
	private boolean isAiming;

	// is the arm in a steady state?
	// if not aiming, always true
	// see AbeDrive for similar truth conditions
	private boolean steady;

	// slides... //

	// slides length for aiming at point
	private double aimSlidesLength;

	private double lastSlidesError;

	private boolean doSlidesAim;

	// position to go to when slides are told not to aim
	private double slidesRestingPosition = AbeConstants.SLIDE_BASE_LENGTH_INCHES + 0.75;

	// elbow... //
	private boolean doElbowAim;

	// elbow angle for aiming at point
	private double aimElbowAngle;

	private double lastElbowError;

	// wrist... //
	private double wristAngle = 0; // relative to elbow, in degrees

	// fingers ... //
	private boolean fingersClamped;

	public AbeArm(AngleAdjuster elbow, LinearSlidesEx slides, PositionableServo wrist, PositionableServo fingers){
		this.elbow = elbow;
		this.slides = slides;
		this.wrist = wrist;
		this.fingers = fingers;

		this.resetSlidesRestingPosition();

		this.timer = new ElapsedTime();
	}

	// GETTERS & SETTERS //

	public void setSlidesRestingPosition(double restingPosition){
		this.slidesRestingPosition = restingPosition;
	}

	public void resetSlidesRestingPosition(){
		this.slidesRestingPosition = AbeConstants.SLIDE_BASE_LENGTH_INCHES + 0.75;
	}

	public boolean isSteady(){
		return this.steady;
	}

	/**
	 * @brief get the current height of the arm
	 *
	 * @return
	 */
	public double getHeight(){
		double elbowAngle = this.elbow.getAngleRadians();
		double slidesExtension = this.slides.getExtension();

		return Math.sin(elbowAngle + Math.atan(AbeConstants.ELBOW_RADIUS_INCHES / slidesExtension)) * Math.sqrt(slidesExtension*slidesExtension + AbeConstants.ELBOW_RADIUS_INCHES*AbeConstants.ELBOW_RADIUS_INCHES);
	}

	public void aimAt(double x, double y){
		this.aimAt(x, y, true, true);
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
	public void aimAt(double x, double y, boolean doElbow, boolean doSlides){
		this.doElbowAim = doElbow;
		this.doSlidesAim = doSlides;

		// set slides extension
		// TODO: account for slide bending?
		this.aimSlidesLength = Math.sqrt(
						x*x + y*y - AbeConstants.ELBOW_RADIUS_INCHES*AbeConstants.ELBOW_RADIUS_INCHES
		);

		// set elbow angle
		this.aimElbowAngle = Math.atan(y/x) - Math.atan(AbeConstants.ELBOW_RADIUS_INCHES / this.aimSlidesLength);

		// determine expected angular adjustment to account for falloff from stress
		double stressFactor = Math.cos(this.aimElbowAngle)*this.aimSlidesLength - AbeConstants.ELBOW_TORQUE_FACTOR*AbeConstants.ELBOW_RADIUS_INCHES*Math.sin(this.aimElbowAngle);

		double falloffCounter = AbeConstants.getExpectedElbowSag(stressFactor);

		// adjust wrist for falloff counter
		this.positionWristRadians(-falloffCounter);

		this.aimElbowAngle += falloffCounter;

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
		/*GlobalStorage.globalTelemetry.addData("wristAngle", this.wristAngle);
		GlobalStorage.globalTelemetry.addData("wrist max range", this.wrist.getMaxRangeDegrees());
		GlobalStorage.globalTelemetry.addData("elbow angle", this.elbow.getAngleDegrees());
		GlobalStorage.globalTelemetry.update();*/

		// this is hacky, but I don't have time to fix this problem at the source
		// sometimes, the wrist angle will be set to NaN by aimAt
		if(Double.isNaN(this.wristAngle)) {
			this.wrist.rotateToDegrees(-this.wrist.getMaxRangeDegrees() / 2. + this.elbow.getAngleDegrees());
		} else {
			this.wrist.rotateToDegrees(-this.wrist.getMaxRangeDegrees() / 2. + this.elbow.getAngleDegrees() + this.wristAngle);
		}
	}

	/**
	 * @brief Update the entire arm (required in automatic mode)
	 */
	public void update(){
		// get delta
		double delta = this.timer.seconds();
		this.timer.reset();

		// update slides length/elbow angle+
		if(this.isAiming() && !this.isManualControlEnabled()){
			// falloff from 55 degrees/second at low extension (<24 inches) to 35 degrees/second at high extension (>36 inches)
			double maxSpeed = Math.toRadians(55.0);
			double minSpeed = Math.toRadians(35.0);

			double minExtension = 24.0;
			double maxExtension = 36.0;

			double interpolation = (Range.clip(this.slides.getExtension(), minExtension, maxExtension) - minExtension) / (maxExtension - minExtension);

			double speed = (1-interpolation) * (maxSpeed - minSpeed) + minSpeed;

			if(this.doElbowAim) this.elbow.rotateToRadians(this.aimElbowAngle, speed);

			double slidesLength = this.aimSlidesLength;
			double slideSpeed = 30.0;

			if(this.doSlidesAim){
				this.slides.extendTo(this.aimSlidesLength, 30.0);
			} else {
				slidesLength = this.slidesRestingPosition;
				this.slides.extendTo(slidesLength, 30.0);
			}

			// check steady state

			// calculate error
			double elbowError = this.aimElbowAngle - this.elbow.getAngleRadians();
			double slidesError = slidesLength - this.slides.getExtension();

			// calculate derivative
			double elbowDerivative = (elbowError - this.lastElbowError) / delta;
			double slidesDerivative = (slidesError - this.lastSlidesError) / delta;

			/*GlobalStorage.globalTelemetry.addData("elbowError", elbowError);
			GlobalStorage.globalTelemetry.addData("elbowDerivative", elbowDerivative);
			GlobalStorage.globalTelemetry.addData("slidesError", slidesError);
			GlobalStorage.globalTelemetry.addData("slidesDerivative", slidesDerivative);*/

			this.steady =
							( Math.abs(elbowError) < AbeArm.ELBOW_STEADY_STATE_ERROR_TOLERANCE && Math.abs(elbowDerivative) < AbeArm.ELBOW_STEADY_STATE_DERIVATIVE_TOLERANCE) &&
							( Math.abs(slidesError) < AbeArm.SLIDES_STEADY_STATE_ERROR_TOLERANCE && Math.abs(slidesDerivative) < AbeArm.SLIDES_STEADY_STATE_DERIVATIVE_TOLERANCE);

			this.lastElbowError = elbowError;
			this.lastSlidesError = slidesError;
		} else {
			this.steady = true;
		}

		// check elbow
		//this.elbow.check();

		// check slides
		this.slides.checkForBadExtension();

		// update wrist
		this.updateWrist();
	}
}
