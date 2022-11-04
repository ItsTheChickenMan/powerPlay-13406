package org.firstinspires.ftc.teamcode.lib.motion;

public class LinearSlides {
	private PositionableMotor driveMotor;

	private double spoolRadius;
	private double spoolCircumference;
	private double retractedLength;
	private double maxLength;

	/**
	 * @brief Create a new set of linear slides
	 *
	 * @param driveMotor
	 * @param retractedLength
	 * @param maxLength
	 * @param spoolRadius
	 */
	public LinearSlides(PositionableMotor driveMotor, double retractedLength, double maxLength, double spoolRadius) {
		this.driveMotor = driveMotor;
		this.retractedLength = retractedLength;
		this.maxLength = maxLength;
		this.spoolRadius = spoolRadius;
		this.spoolCircumference = 2*Math.PI*this.spoolRadius;
	}

	/**
	 * @brief Returns the total amount that this slide is extended, even through encoder resets
	 */
	public double getExtension(){
		return this.spoolCircumference * this.driveMotor.getRotations() + this.retractedLength;
	}

	public void extend(double inches, double velocity){
		double target = this.getExtension() + inches;

		this.extendTo(target, velocity);
	}

	/**
	 * @brief extend to a certain number of inches from the point of mounting (it accounts for the retracted length)
	 *
	 * @param inches inches to extend to
	 * @param velocity inches / second
	 */
	public void extendTo(double inches, double velocity) {
		// validate
		if(inches > this.maxLength || inches < this.retractedLength){
			return;
		}

		// calculate rotations
		double rotations = (inches - this.retractedLength) / this.spoolCircumference;
		double rotationsVelocity = velocity / this.spoolCircumference;

		// drive
		this.driveMotor.rotateTo(rotations, rotationsVelocity);
	}

	/**
	 * @brief Set the speed of the extension (not any particular length)
	 *
	 * This still applies the restraints to length that the others apply, and silently fails if it predicts over or under extension
	 * However, this is still technically unsafe since it does not actively apply restraints, since it can only predict so far.
	 *
	 * @param velocity inches / second
	 */
	public void setVelocity(double velocity){
		if( (this.getExtension() > this.maxLength && velocity > 0) || (this.getExtension() < this.retractedLength && velocity < 0) ){
			this.driveMotor.rotateSpeed(0);
			return;
		}

		// calculate rotations
		double rotationsVelocity = velocity / this.spoolCircumference;

		this.driveMotor.rotateSpeed(rotationsVelocity);
	}

	/**
	 * @brief Checks if the slides are over or under extending, and stops them if they are.
	 *
	 * It's generally good to call this in the update loop, just in case.
	 *
	 * @param correctionSpeed the speed, in inches/second, to correct the slides' positions if necessary.
	 */
	public void checkForBadExtension(double correctionSpeed){
		if(this.getExtension() > this.maxLength) {
			this.extendTo(this.maxLength - 0.1, correctionSpeed);
		} else if(this.getExtension() < this.retractedLength){
			this.extendTo(this.retractedLength + 0.1, correctionSpeed);
		}
	}
}
