package org.firstinspires.ftc.teamcode.lib.motion;

public class LinearSlides {
	private PositionableMotor driveMotor;

	private double rotationsUntilFull;
	private double reach;

	public LinearSlides(PositionableMotor driveMotor, double reach, double rotationsUntilFull) {
		this.driveMotor = driveMotor;
		this.reach = reach;
		this.rotationsUntilFull = rotationsUntilFull;
	}

	/**
	 * @brief extend to a certain number of inches
	 *
	 * @FIXME tends to be off by a small amount (0.25-0.5 inches)
	 *
	 * @param inches inches to extend to
	 * @param velocity inches / second
	 */
	public void extendTo(double inches, double velocity) {
		double percent = inches / reach;

		double rotations = percent * rotationsUntilFull;

		double percentVelocity = velocity / reach;

		double rotationsVelocity = percentVelocity * rotationsUntilFull;

		this.driveMotor.rotate(rotations, rotationsVelocity);
	}
}
