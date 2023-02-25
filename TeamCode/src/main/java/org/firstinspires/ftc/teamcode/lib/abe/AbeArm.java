package org.firstinspires.ftc.teamcode.lib.abe;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.motion.LinearSlides;
import org.firstinspires.ftc.teamcode.lib.motion.PositionableMotor;
import org.firstinspires.ftc.teamcode.lib.utils.JunctionHelper;
import org.firstinspires.ftc.teamcode.lib.utils.MathUtils;
import org.firstinspires.ftc.teamcode.lib.utils.PolynomialRegression;

/**
 * @brief Class for managing the abe arm system (elbow, slides, wrist, claw)
 */
public class AbeArm {
	// hardware... //
	private PositionableMotor elbowMotor;

	private LinearSlides slides;

	private AbeHand hand;

	// delta timer //
	private ElapsedTime deltaTimer;

	// aim details... //
	private double elbowAngleRadians;
	private double slidesLengthInches;

	private double desiredWristAngleRadians;

	private double slidesRestingExtensionInches;

	// sag correction details
	private PolynomialRegression[] sagCorrectionRegressions;

	// steady state details
	private double elbowSteadyStateMaximumErrorRadians = Math.toRadians(1.0);
	private double elbowSteadyStateMaximumDerivativeRadians = Math.toRadians(2.0);

	private double slidesSteadyStateMaximumErrorInches = 0.75;
	private double slidesSteadyStateMaximumDerivativeInches = 1.0;

	private double elbowLastErrorRadians = 0.0;
	private double elbowLastDerivativeRadians = 0.0;

	private double slidesLastErrorInches = 0.0;
	private double slidesLastDerivativeInches = 0.0;

	public AbeArm(PositionableMotor elbowMotor, LinearSlides slides, AbeHand hand, Vector2d[][] sagCorrectionPoints){
		this.slides = slides;
		this.elbowMotor = elbowMotor;
		this.hand = hand;

		this.slidesRestingExtensionInches = this.slides.getBaseExtension() + 0.5;

		this.sagCorrectionRegressions = new PolynomialRegression[4];

		// create polynomial regressions for sag correction
		for(int i = 0; i < sagCorrectionPoints.length; i++){
			Vector2d[] level = sagCorrectionPoints[i];

			if(level.length < 1) continue;

			double[] x = new double[level.length];
			double[] y = new double[level.length];

			for(int j = 0; j < level.length; j++){
				x[j] = level[j].getX();
				y[j] = level[j].getY();
			}

			// in most cases, quadratic works
			if(i < 3){
				this.sagCorrectionRegressions[i] = new PolynomialRegression(x, y, 2);
			}
			// for highest level, cubic is a bit better (accounts for less correction needed when angled high)
			else {
				this.sagCorrectionRegressions[i] = new PolynomialRegression(x, y, 3);
			}
		}

		this.deltaTimer = new ElapsedTime();
	}

	public double getSagCorrectionR2(JunctionHelper.Level level){
		PolynomialRegression r = this.sagCorrectionRegressions[JunctionHelper.getJunctionIndex(level)];

		if(r == null) return -1.0;

		return r.R2();
	}

	public void setDefaultElbowSteadyStateMaximumErrorRadians(double radians){
		this.elbowSteadyStateMaximumErrorRadians = radians;
	}

	public void setDefaultElbowSteadyStateMaximumDerivativeRadians(double radians){
		this.elbowSteadyStateMaximumDerivativeRadians = radians;
	}

	public void setDefaultSlidesSteadyStateMaximumErrorInches(double inches){
		this.slidesSteadyStateMaximumErrorInches = inches;
	}

	public void setDefaultSlidesSteadyStateMaximumDerivativeInches(double inches){
		this.slidesSteadyStateMaximumDerivativeInches = inches;
	}

	public void setElbowOffsetRadians(double offset){
		this.elbowMotor.setAngleOffsetRadians(offset);
	}

	public void setElbowOffsetDegrees(double offset){
		this.elbowMotor.setAngleOffsetDegrees(offset);
	}

	public void setSlidesExtensionOffsetInches(double offset){
		this.slides.setExtensionOffset(offset);
	}

	public double getSlidesBaseExtension(){
		return this.slides.getBaseExtension();
	}

	/**
	 * @brief check if elbow is steady using the default tolerances
	 *
	 * @return true if elbow is steady (within a certain error and speed threshold)
	 */
	public boolean isElbowSteady(){
		return this.isElbowSteady(this.elbowSteadyStateMaximumErrorRadians, this.elbowSteadyStateMaximumDerivativeRadians);
	}

	/**
	 * @brief check if elbow is steady using custom tolerances
	 *
	 * @param maxErrorRadians
	 * @param maxDerivativeRadians
	 * @return true if elbow is steady (within a certain error and speed threshold)
	 */
	public boolean isElbowSteady(double maxErrorRadians, double maxDerivativeRadians){
		return (this.elbowLastErrorRadians <= maxErrorRadians) && (this.elbowLastDerivativeRadians <= maxDerivativeRadians);
	}

	/**
	 * @brief check if slides are steady using the default tolerances
	 *
	 * @return true if slides are steady (within a certain error and speed threshold)
	 */
	public boolean areSlidesSteady(){
		return this.areSlidesSteady(this.slidesSteadyStateMaximumErrorInches, this.slidesSteadyStateMaximumDerivativeInches);
	}

	/**
	 * @brief check if slides are steady using custom tolerances
	 *
	 * @param maxErrorInches
	 * @param maxDerivativeInches
	 * @return true if slides are steady (within a certain error and speed threshold)
	 */
	public boolean areSlidesSteady(double maxErrorInches, double maxDerivativeInches){
		return (this.slidesLastErrorInches <= maxErrorInches) && (this.slidesLastDerivativeInches <= maxDerivativeInches);
	}

	/**
	 * @brief check if arm is steady (elbow and slides are steady) using default tolerances
	 *
	 * @return true if both elbow and slides are steady
	 */
	public boolean isSteady(){
		return this.isElbowSteady() && this.areSlidesSteady();
	}

	/**
	 * @return true if the elbow is currently running to position (doesn't use steady state settings)
	 */
	public boolean isElbowBusy(){
		return this.elbowMotor.isBusy();
	}

	/**
	 * @return true if the slides are currently running to position (doesn't use steady state settings)
	 */
	public boolean areSlidesBusy(){
		return this.slides.isBusy();
	}

	public double getSlidesRestingExtensionInches(){
		return this.slidesRestingExtensionInches;
	}

	/**
	 * @param length inches, must be greater than base extension and less than max extension
	 * @return set the resting position (when told not to aim) of the slides
	 */
	public void setSlidesRestingExtensionInches(double length){
		if(length < this.slides.getBaseExtension() || length > this.slides.getMaxExtension()) return;

		this.slidesRestingExtensionInches = length;
	}

	/**
	 * @brief get the currently desired wrist rotation in radians
	 */
	public double getDesiredWristAngleRadians(){
		return this.desiredWristAngleRadians;
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
		this.desiredWristAngleRadians = angle;
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
	 * @return the current (not intended) angle of the elbow in radians
	 */
	public double getElbowAngleRadians(){
		return this.elbowMotor.getAngleRadians();
	}

	/**
	 * @return the current (not intended) angle of the elbow in degrees
	 */
	public double getElbowAngleDegrees(){
		return this.elbowMotor.getAngleDegrees();
	}

	/**
	 * @brief manually set the elbow angle to some value in radians
	 */
	public void setElbowAngleRadians(double elbowAngle){
		this.elbowAngleRadians = elbowAngle;
	}

	/**
	 * @brief manually set the elbow angle to some value in degrees
	 */
	public void setElbowAngleDegrees(double elbowAngle){
		this.setElbowAngleRadians(Math.toRadians(elbowAngle));
	}

	/**
	 * @brief makes the elbow go the speed specified in radians/s.  do not use in conjunction with update(), it'll either not work at all or mess things up
	 *
	 * @param velocityRadians
	 */
	public void setElbowVelocityRadians(double velocityRadians){
		this.elbowMotor.rotateSpeedRadians(velocityRadians);
	}

	/**
	 * @brief see setElbowVelocityRadians
	 *
	 * @param velocityDegrees
	 */
	public void setElbowVelocityDegrees(double velocityDegrees){
		this.setElbowVelocityRadians(Math.toRadians(velocityDegrees));
	}

	/**
	 * @return the current (not intended) extension of the slides, in inches
	 */
	public double getSlidesLengthInches(){
		return this.slides.getExtension();
	}

	/**
	 * @brief manually set slides length to some value in inches
	 */
	public void setSlidesLengthInches(double slidesLength){
		this.slidesLengthInches = slidesLength;
	}

	/**
	 * @brief makes the slides go the speed specified in in/s.  do not use in conjunction with update(), it'll either not work at all or mess things up
	 *
	 * @param velocityInches
	 */
	public void setSlidesVelocityInches(double velocityInches){
		this.slides.setVelocity(velocityInches);
	}

	public double calculateElbowSagCorrectionRadians(double distance, double height){
		// get low + high junction levels
		// goal is to encapsulate value and then linearly interpolate between two estimates to get best result
		JunctionHelper.Level lowLevel = JunctionHelper.Level.NONE;
		JunctionHelper.Level highLevel = JunctionHelper.Level.NONE;

		for(int i = 0; i < JunctionHelper.JUNCTION_LEVELS.length; i++){
			JunctionHelper.Level level = JunctionHelper.getJunctionLevelFromIndex(i);

			double h = JunctionHelper.getJunctionHeight(level);

			if(h > height){
				// assign high level
				highLevel = level;

				// get next lowest level
				lowLevel = JunctionHelper.getJunctionLevelFromIndex(i-1);
			}
		}

		// if no higher value, just use high
		if(highLevel == JunctionHelper.Level.NONE) highLevel = JunctionHelper.Level.HIGH;

		// if no lower value, just use ground
		if(lowLevel == JunctionHelper.Level.NONE) lowLevel = JunctionHelper.Level.GROUND;

		double lowHeight = JunctionHelper.getJunctionHeight(lowLevel);
		double highHeight = JunctionHelper.getJunctionHeight(highLevel);

		int lowIndex = JunctionHelper.getJunctionIndex(lowLevel);
		int highIndex = JunctionHelper.getJunctionIndex(highLevel);

		// check if we require interpolation
		if(MathUtils.approximatelyEqual(highHeight, height, 0.5)){
			// no interpolation needed

			// get regression
			PolynomialRegression r = this.sagCorrectionRegressions[highIndex];

			// if null, return 0
			if(r == null) return 0.0;

			// get estimated value
			double predicted = r.predict(distance);

			// return
			return predicted;
		} else {
			// get two regressions
			PolynomialRegression lowRegression = this.sagCorrectionRegressions[lowIndex];
			PolynomialRegression highRegression = this.sagCorrectionRegressions[highIndex];

			// if either null, return 0
			if(lowRegression == null || highRegression == null) return 0.0;

			// get two predictions
			double lowPrediction = lowRegression.predict(distance);
			double highPrediction = highRegression.predict(distance);

			// estimate final predictions
			double predicted = MathUtils.map(height, lowHeight, highHeight, lowPrediction, highPrediction);

			// return
			return predicted;
		}
	}

	/**
	 * @brief Calculate the slides length required to aim at a point (x, y) in arm space
	 *
	 * @param x x coordinate in arm space
	 * @param y y coordinate in arm space
	 */
	public double calculateSlidesLengthInches(double x, double y){
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
	public double calculateElbowAngleRadians(double x, double y){
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
		double elbowAngle = this.calculateElbowAngleRadians(x, y);
		double slidesLength = this.calculateSlidesLengthInches(x, y);

		// calculate correction
		double sagCorrection = this.calculateElbowSagCorrectionRadians(x, y);

		elbowAngle += sagCorrection;

		// save values
		this.elbowAngleRadians = elbowAngle;
		this.slidesLengthInches = slidesLength;
	}

	public void updateHand(){
		// do hand
		double actualWristAngle = this.desiredWristAngleRadians - this.elbowMotor.getAngleRadians();

		this.hand.setPitchRadians(actualWristAngle);
	}

	public void updateSteadyState(){
		double delta = this.deltaTimer.seconds();
		deltaTimer.reset();

		// steady state calculation
		double elbowError = this.getElbowAngleRadians() - this.elbowAngleRadians;
		double elbowDerivative = (elbowError - this.elbowLastErrorRadians) / delta;
		this.elbowLastErrorRadians = elbowError;
		this.elbowLastDerivativeRadians = elbowDerivative;

		double slidesError = this.getSlidesLengthInches() - this.slidesLengthInches;
		double slidesDerivative = (slidesError - this.slidesLastErrorInches) / delta;
		this.slidesLastErrorInches = slidesError;
		this.slidesLastDerivativeInches = slidesDerivative;
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
			this.elbowMotor.rotateToRadians(this.elbowAngleRadians, AbeConstants.ELBOW_VELOCITY_RADIANS);
		}

		// aim slides, if told to
		if(doSlides){
			this.slides.extendTo(this.slidesLengthInches, AbeConstants.SLIDES_VELOCITY_INCHES);
		} else{
			// go to default position
			this.slides.extendTo(this.slidesRestingExtensionInches, AbeConstants.SLIDES_VELOCITY_INCHES);
		}

		this.updateHand();

		this.updateSteadyState();
	}
}
