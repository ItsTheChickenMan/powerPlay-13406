package org.firstinspires.ftc.teamcode.lib.Abe;

import org.firstinspires.ftc.teamcode.lib.motion.AngleAdjuster;
import org.firstinspires.ftc.teamcode.lib.motion.LinearSlides;
import org.firstinspires.ftc.teamcode.lib.motion.PositionableMotor;
import org.firstinspires.ftc.teamcode.lib.motion.PositionableServo;

/**
 * @brief The entire arm (angle adjuster, slides, wrist, and fingers) wrapped into one easy-to-use class
 */
public class Arm {
	/**
	 * @brief for the sake of time, we'll allow elbow to be controlled from outside the class since it's not necessary to encapsulate it
	 */
	public AngleAdjuster elbow;
	public LinearSlides slides;

	private PositionableServo wrist;
	private PositionableServo fingers;

	public Arm(AngleAdjuster elbow, LinearSlides slides, PositionableServo wrist, PositionableServo fingers){
		this.elbow = elbow;
		this.slides = slides;
		this.wrist = wrist;
		this.fingers = fingers;
	}

	/**
	 * @brief Rotate the wrist to a certain orientation, relative to the elbow's orientation
	 *
	 * @param angle angle, in degrees
	 */
	public void positionWristDegrees(double angle){
		this.wrist.rotateToDegrees(angle - this.elbow.getAngleDegrees());
	}

	/**
	 * @brief Rotate the wrist to a certain orientation, relative to the elbow's orientation
	 *
	 * @param angle, in radians
	 */
	public void positionWristRadians(double angle){
		this.wrist.rotateToRadians(angle - this.elbow.getAngleRadians());
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
}
