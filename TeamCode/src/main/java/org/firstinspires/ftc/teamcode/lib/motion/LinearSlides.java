package org.firstinspires.ftc.teamcode.lib.motion;

import com.qualcomm.robotcore.hardware.DcMotor;

public class LinearSlides {
	private PositionableMotor driveMotor;

	private double spoolRadius;
	private double spoolCircumference;
	private double retractedLength;
	private double maxLength;

	private boolean disableRetraction;
	private boolean disableExtension;

	// position to freeze at
	private boolean frozen;
	private double frozenRotations;

	// the desired velocity.  not guaranteed to be the actual velocity.  only used to check for over/under extension, which is why it has no getter/setter
	private double currentDesiredVelocity;

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
	 * @brief Stop the slides from moving
	 *
	 * @todo PositionableMotor method?
	 */
	public void freeze(){
		if(!this.frozen){
			this.frozenRotations = this.driveMotor.getRotations();
		}

		this.frozen = true;

		this.driveMotor.rotateTo(this.frozenRotations, 1.0);
	}

	/**
	 * @brief Allow the slides to move again
	 *
	 * @todo PositionableMotor method?
	 */
	public void unfreeze(){
		this.driveMotor.resetEncoders();

		this.frozen = false;
	}

	/**
	 * @brief Returns the current velocity of extension
	 *
	 * @return
	 */
	public double getVelocity(){
		return this.spoolCircumference * this.driveMotor.getVelocityRotations();
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
	 * @deprecated the method is supposed to account for
	 *
	 * This still applies the restraints to length that the others apply, and silently fails if it predicts over or under extension
	 * However, this is still technically unsafe since it does not actively apply restraints, since it can only predict so far.
	 *
	 * @param velocity inches / second
	 */
	public void setVelocity(double velocity){
		if( (velocity > 0 && this.disableExtension) || (velocity < 0 && this.disableRetraction) ){
			return;
		}

		currentDesiredVelocity = velocity;

		// calculate rotations
		double rotationsVelocity = velocity / this.spoolCircumference;

		this.driveMotor.rotateSpeed(rotationsVelocity);
	}

	/**
	 * @brief Checks if the slides are over or under extending, and stops them if they are.
	 *
	 * @deprecated don't use this, I'm probably going to remove it later
	 *
	 * Call this in the update loop to prevent the slides from over/under extending due to setVelocity
	 */
	public void checkForBadExtension(){
		if(this.getExtension() > this.maxLength && currentDesiredVelocity > 0) {
			this.disableExtension = true;
		} else {
			//this.unfreeze();
			this.disableExtension = false;
		}

		if(this.getExtension() < this.retractedLength && currentDesiredVelocity < 0){
			this.disableRetraction = true;
		} else {
			//this.unfreeze();
			this.disableRetraction = false;
		}
	}
}
