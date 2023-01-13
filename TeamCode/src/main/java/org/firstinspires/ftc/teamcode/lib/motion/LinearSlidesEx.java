package org.firstinspires.ftc.teamcode.lib.motion;

/**
 * @brief Allows slide control with multiple motors
 */
public class LinearSlidesEx {
	// multiple motors can power the same set of slides
	// it assumes that all have the same gear ratio
	private LinearSlides[] slides;

	public LinearSlidesEx(PositionableMotor[] motors, double retractedLength, double maxLength, double spoolCircumference, double extensionFactor) throws Exception {
		// ensure there's enough motors
		if(motors.length < 1){
			throw new Exception("requires at least one motor");
		}

		this.slides = new LinearSlides[motors.length];

		for(int i = 0; i < motors.length; i++){
			PositionableMotor motor = motors[i];

			this.slides[i] = new LinearSlides(motor, retractedLength, maxLength, spoolCircumference, extensionFactor);
		}
	}

	/**
	 * @FIXME this sucks
	 */
	public double getBaseExtension(){
		return this.slides[0].getBaseExtension();
	}

	public double getMaxExtension(){
		return this.slides[0].getMaxExtension();
	}

	public double getTravel(){
		return this.slides[0].getTravel();
	}

	public void setExtensionOffset(double offset){
		for(LinearSlides slide : this.slides){
			slide.setExtensionOffset(offset);
		}
	}

	public void freeze(){
		for(LinearSlides slide : this.slides){
			slide.freeze();
		}
	}

	public void unfreeze(){
		for(LinearSlides slide : this.slides){
			slide.unfreeze();
		}
	}

	/**
	 * @brief Returns the current velocity of extension
	 *
	 * @return
	 */
	public double getVelocity(){
		return this.slides[0].getVelocity();
	}

	/**
	 * @brief returns the total that this slide is extended relative to the starting point, even through encoder resets
	 *
	 * @return
	 */
	public double getRelativeExtension(){
		return this.slides[0].getRelativeExtension();
	}

	/**
	 * @brief Returns the total amount that this slide is extended, even through encoder resets
	 */
	public double getExtension(){
		return this.slides[0].getExtension();
	}

	public void extend(double inches, double velocity){
		for(LinearSlides slide : this.slides){
			slide.extend(inches, velocity);
		}
	}

	/**
	 * @brief extend to a certain number of inches from the point of mounting (it accounts for the retracted length)
	 *
	 * @param inches inches to extend to
	 * @param velocity inches / second
	 */
	public void extendTo(double inches, double velocity) {
		for(LinearSlides slide : this.slides){
			slide.extendTo(inches, velocity);
		}
	}

	public void setVelocity(double velocity){
		// FIXME: implement
		for(LinearSlides slide : this.slides){
			slide.setVelocity(velocity);
		}
	}

	public void checkForBadExtension(){
		// FIXME: implement
	}
}
