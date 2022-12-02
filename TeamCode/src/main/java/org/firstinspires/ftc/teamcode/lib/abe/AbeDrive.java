package org.firstinspires.ftc.teamcode.lib.abe;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.lib.utils.AngleHelper;
import org.firstinspires.ftc.teamcode.lib.utils.Vec2;

/**
 * @brief Drive that is specifically designed for versions of Abe during Power Play
 *
 * Mostly works off of the RR SampleMecanumDrive class, but includes utilities to make it work for the stuff we need it for
 */
public class AbeDrive {
	// ENUMS //
	public static enum AimMode {
		NONE,
		POINT,
		ANGLE
	}

	private SampleMecanumDrive drive;

	// cumulative powers
	private double frontLeftPower;
	private double frontRightPower;
	private double backLeftPower;
	private double backRightPower;

	// current aim mode
	private AimMode aimMode = AimMode.NONE;

	// point being aimed at currently, if mode is POINT
	// FIXME: probably should switch to apache vectors
	private Vector2d aimAtPoint;

	// angle being aimed at currently, if mode is ANGLE
	private double aimAtAngle;

	// PID stuff
	private double aim_p;
	private double aim_i;
	private double aim_d;

	private double lastError = 0;
	private double totalError = 0;

	private ElapsedTime deltaTimer;

	// other settings
	private double armOffset;

	public AbeDrive(HardwareMap hardwareMap, double armOffset, double aim_p, double aim_i, double aim_d){
		this.drive = new SampleMecanumDrive(hardwareMap);
		this.deltaTimer = new ElapsedTime();

		this.armOffset = armOffset;

		// assing pid vals
		this.aim_p = aim_p;
		this.aim_i = aim_i;
		this.aim_d = aim_d;

		// no aim initially
		this.aimMode = AimMode.NONE;

		// clear out powers just for fun
		this.clearCumulativePowers();
	}

	/**
	 * @brief mirror for SampleMecanumDrive's getPoseEstimate
	 *
	 * @return current pose estimate (see RR docs)
	 */
	public Pose2d getPoseEstimate(){
		return this.drive.getPoseEstimate();
	}

	public Vector2D getPoseEstimateAsVector(){
			return new Vector2D(this.drive.getPoseEstimate().getX(), this.drive.getPoseEstimate().getY());
	}

	/**
	 * @brief Set the pose estimate for the drive train,
	 *
	 * @param x
	 * @param y
	 * @param rotation
	 */
	public void setPoseEstimate(double x, double y, double rotation){
		this.drive.setPoseEstimate(new Pose2d(
			x, y, rotation
		));
	}

	public void setPoseEstimate(Pose2d pose){
		this.drive.setPoseEstimate(pose);
	}

	/**
	 * @brief is the bot aiming at a point at the moment?
	 *
	 * @return true if it is, false if it isn't
	 */
	public boolean isAiming(){
		return this.aimMode != AimMode.NONE;
	}

	/**
	 * @brief set the bot to aim at a point, where the point is relative to the starting place of the robot
	 *
	 * This overwrites any point that was there before
	 *
	 * @note uses RR axes (which are weird, look it up)
	 *
	 * @param x x value for point to aim at
	 * @parma y y value for point to aim at
	 */
	public void aimAtPoint(double x, double y){
		this.aimMode = AimMode.POINT;

		this.aimAtPoint = new Vector2d(x, y);
	}

	/**
	 * @brief aim at an angle, in degrees
	 *
	 * @param angle
	 */
	public void aimAtAngleDegrees(double angle){
		this.aimAtAngleRadians(Math.toRadians(angle));
	}

	/**
	 * @brief aim at an angle, in radians
	 *
	 * @param angle
	 */
	public void aimAtAngleRadians(double angle){
		this.aimMode = AimMode.ANGLE;

		this.aimAtAngle = angle;
	}

	/**
	 * @brief locks the drive's rotation onto its current heading until another call to an aimAt method or clearAim
	 */
	public void aimAtCurrentAngle(){
		// fetch angle from current pose
		double rotation = this.drive.getPoseEstimate().getHeading();

		this.aimAtAngleRadians(rotation);
	}

	/**
	 * @brief clears current aim, despite mode
	 */
	public void clearAim(){
		this.aimMode = AimMode.NONE;
	}

	/**
	 * @brief Mirror for SampleMecanumDrive setMotorPowers, but uses parameter order that's better IMO
	 *
	 */
	private void setMotorPowers(double frontLeft, double frontRight, double backLeft, double backRight){
		this.drive.setMotorPowers(frontLeft, backLeft, backRight, frontRight);
	}

	/**
	 * @brief Clears out the currently accumulated powers.  does not set the powers to the motor; that's what flushCumulativePowers is for
	 */
	private void clearCumulativePowers(){
		this.frontLeftPower = 0;
		this.frontRightPower = 0;
		this.backLeftPower = 0;
		this.backRightPower = 0;
	}

	/**
	 * @brief Flushes out the accumulated powers to the motors
	 */
	private void flushCumulativePowers(){
		this.setMotorPowers(this.frontLeftPower, this.frontRightPower, this.backLeftPower, this.backRightPower);

		this.clearCumulativePowers();
	}

	/**
	 * @brief sets the motor powers according to field oriented drive rules and desired forward/strafe movement
	 *
	 * Does not consider rotation because the abe drive handles its own rotation
	 *
	 * Powers do not take affect until update() is called
	 *
	 * @param forward desired forward/backward movement, between -1 and 1 (enforced)
	 * @param strafe desired left/right movement, between -1 and 1 (enforced)
	 */
	public void driveFieldOriented(double forward, double strafe){
		// clamp values
		forward = Math.min(Math.max(forward, -1.0), 1.0);
		strafe = Math.min(Math.max(strafe, -1.0), 1.0);

		// convert to vector
		// TODO: should probably switch to RR vectors
		Vec2 driveVec = new Vec2(strafe, forward);

		// TODO: is this expensive to call here?
		// get heading
		Pose2d pose = this.drive.getPoseEstimate();
		double heading = pose.getHeading();

		// rotate opposite of heading
		driveVec.rotate(-heading);

		forward = driveVec.y;
		strafe = driveVec.x;

		// assign powers
		this.frontLeftPower += forward + strafe;
		this.frontRightPower += forward - strafe;
		this.backLeftPower += forward - strafe;
		this.backRightPower += forward + strafe;
	}

	public void rotate(double rotation){
		this.frontLeftPower += rotation;
		this.frontRightPower -= rotation;
		this.backLeftPower += rotation;
		this.backRightPower -= rotation;
	}

	public void normalizePowers(){
		double largest = Math.max(this.backRightPower, Math.max(this.backLeftPower, Math.max(this.frontLeftPower, this.frontRightPower)));

		if(largest > 1.0){
			this.frontLeftPower /= largest;
			this.frontRightPower /= largest;
			this.backLeftPower /= largest;
			this.backRightPower /= largest;
		}
	}

	/**
	 * @brief Update drivetrain odometry and point/angle being aimed at
	 */
	public void update(){
		this.update(false);
	}

	/**
	 * @brief Update drivetrain odometry and point/angle being aimed at
	 *
	 * @param dontDoAim if true, doesn't do aim despite the current setting
	 */
	public void update(boolean dontDoAim){
		// update odometry
		this.drive.update();

		// get delta
		double delta = this.deltaTimer.seconds();

		// get current pose
		Pose2d pose = this.drive.getPoseEstimate();
		Vector2d coords = pose.vec();
		double heading = pose.getHeading();

		if(this.isAiming() && !dontDoAim){
			double desiredAngle = 0;

			if(this.aimMode == AimMode.POINT) {
				Vector2d offset = coords.minus(this.aimAtPoint);

				double distance = offset.norm();

				if (distance != 0.0) {
					// get currently desired angle
					desiredAngle = Math.atan2(offset.getY(), offset.getX()) + Math.asin(this.armOffset / distance);
					desiredAngle += Math.PI;
				}
			} else if(this.aimMode == AimMode.ANGLE){
				desiredAngle = this.aimAtAngle;
			}

			// pid
			// TODO: abstract into class/borrow a better implementation from somewhere

			// get error
			double error = -AngleHelper.angularDistanceRadians(heading, desiredAngle);

			// get error derivative
			double derivative = (error - this.lastError) / delta;

			// save error
			this.lastError = error;

			// add to integral
			if (!Double.isNaN(error)) this.totalError += error * delta;

			// calculate rotation power
			double rotation = error * this.aim_p + derivative * this.aim_d + this.totalError * this.aim_i;

			// FIXME: remove this hack
			double maxRotation = 0.5;
			rotation = Math.max(Math.min(rotation, maxRotation), -maxRotation);

			if (!Double.isNaN(rotation)) {
				// add rotation to cumulative powers
				this.frontLeftPower += rotation;
				this.frontRightPower -= rotation;
				this.backLeftPower += rotation;
				this.backRightPower -= rotation;
			}
		}

		this.normalizePowers();

		// flush powers to motors
		this.flushCumulativePowers();

		// reset delta timer
		deltaTimer.reset();
	}
}
