package org.firstinspires.ftc.teamcode.lib.motion;

public class LinearSlides {
	private PositionableMotor driveMotor;

	private double spoolRadius;
	private double spoolCircumference;
	private double retractedLength;
	private double maxLength;

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
		return this.spoolCircumference * this.driveMotor.getRotations();
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
		double rotations = inches / this.spoolCircumference;
		double rotationsVelocity = velocity / this.spoolCircumference;

		// drive
		this.driveMotor.rotate(rotations, rotationsVelocity);
	}
}
