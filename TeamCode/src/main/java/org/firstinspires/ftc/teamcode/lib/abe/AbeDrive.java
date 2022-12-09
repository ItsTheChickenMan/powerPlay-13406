package org.firstinspires.ftc.teamcode.lib.abe;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.lib.utils.AngleHelper;
import org.firstinspires.ftc.teamcode.lib.utils.GlobalStorage;
import org.firstinspires.ftc.teamcode.lib.utils.Vec2;

/**
 * @brief Drive that is specifically designed for versions of Abe during Power Play
 *
 * Mostly works off of the RR SampleMecanumDrive class, but includes utilities to make it work for the stuff we need it for
 */
public class AbeDrive {
	// enum for current aiming system that the drive train is using
	public enum AimMode {
		NONE,
		POINT,
		ANGLE
	}

	public static final double STEADY_STATE_ERROR_TOLERANCE = Math.toRadians(3.5);
	public static final double STEADY_STATE_DERIVATIVE_TOLERANCE = Math.toRadians(0.1);

	// rr interface
	private SampleMecanumDrive drive;

	// cumulative powers
	private double frontLeftVelocity;
	private double frontRightVelocity;
	private double backLeftVelocity;
	private double backRightVelocity;

	// are we in a steady state?
	// always true if not aiming
	// if aiming, true if err is below STEADY_STATE_ERROR_TOLERANCE and derr/dt is below STEADY_STATE_DERIVATIVE_TOLERANCE
	private boolean steady;

	// current aim mode
	private AimMode aimMode;

	// point being aimed at currently, if mode is POINT
	// FIXME: probably should switch to apache vectors
	private Vector2d aimAtPoint;

	// angle being aimed at currently, if mode is ANGLE
	private double aimAtAngle;

	// PID stuff
	private double aim_p;
	private double aim_i;
	private double aim_d;

	// delta settings
	private double lastError = 0;
	private double totalError = 0;

	private double lastTime;
	private ElapsedTime deltaTimer;

	/*
	private double lastAimUpdate = 0;
	private double aimUpdateRate = 0.75;
	private double aimIntervalRange = 9.0;
	*/

	private double distanceFalloffRange = 12.0;
	private double falloffRate = 1.0;

	private double rotationSpeed = 1.0;

	// position offset
	// I would just set the pose estimate of the RR drive class, but doing so causes the drivetrain to spin in teleop for some reason that's probably out of my control, so I'm doing this instead
	private double positionXOffset = 0;
	private double positionYOffset = 0;

	// other settings
	private double armOffset;

	private double gearRatio;
	private double tickRatio;
	private double wheelCircumference;

	private double maxVelocity;

	public AbeDrive(HardwareMap hardwareMap, double armOffset, double gearRatio, double tickRatio, double wheelCircumference, double maxVelocity, double aim_p, double aim_i, double aim_d) {
		this.drive = new SampleMecanumDrive(hardwareMap);
		this.deltaTimer = new ElapsedTime();

		this.armOffset = armOffset;

		// assing pid vals
		this.aim_p = aim_p;
		this.aim_i = aim_i;
		this.aim_d = aim_d;

		this.gearRatio = gearRatio;
		this.tickRatio = tickRatio;
		this.wheelCircumference = wheelCircumference;
		this.maxVelocity = maxVelocity;

		// no aim initially
		this.aimMode = AimMode.NONE;

		// clear out powers just for fun
		this.clearCumulativeVelocities();
	}

	/**
	 * @param ticks
	 * @return
	 * @brief Returns the amount of inches that the drive train is expected to travel, not accounting for traction, if all wheels were to move a certain number of ticks
	 */
	public double tickToInches(double ticks) {
		// convert ticks to rotations
		double rotations = ticks / this.tickRatio;

		// convert driver gear rotations to driven gear rotations
		rotations *= this.gearRatio;

		// convert rotations to inches travelled
		return rotations * this.wheelCircumference;
	}

	/**
	 * @param inches
	 * @return
	 * @brief Returns the amount of ticks that the drive train is expected to need to travel a certain number of inches, not accounting for traction
	 */
	public double inchesToTicks(double inches) {
		// convert inches to rotations
		double rotations = inches / this.wheelCircumference;

		// convert driven gear rotations to driver gear rotations
		rotations /= this.gearRatio;

		// convert rotations to ticks
		return rotations * this.tickRatio;
	}

	public boolean isSteady(){
		return this.steady;
	}

	/**
	 * @return current pose estimate (see RR docs)
	 * @brief mirror for SampleMecanumDrive's getPoseEstimate
	 */
	public Pose2d getPoseEstimate() {
		Pose2d offset = new Pose2d(this.positionXOffset, this.positionYOffset);

		return offset.plus(this.drive.getPoseEstimate());
	}

	public Vector2D getPoseEstimateAsVector() {
		return new Vector2D(this.getPoseEstimate().getX(), this.getPoseEstimate().getY());
	}


	/**
	 * @brief Get the heading of the robot as a normalized vector
	 *
	 * @return a Vector2D representing the forward vector
	 */
	public Vector2D getForwardVector(){
		// the Vector2D doesn't have a rotation method for some reason, so use RR vectors and then switch
		// I hate this a lot, but I don't really have the energy to switch all of the vectors over
		Vector2d forward = new Vector2d(1, 0).rotated(this.getPoseEstimate().getHeading());

		return new Vector2D(forward.getX(), forward.getY());
	}

	/**
	 * @param x
	 * @param y
	 * @param rotation
	 * @brief Set the pose estimate for the drive train, without affecting the position offsets
	 */
	public void setPoseEstimate(double x, double y, double rotation) {
		this.drive.setPoseEstimate(new Pose2d(
						x, y, rotation
		));
	}

	public void setPoseEstimate(Pose2d pose) {
		this.drive.setPoseEstimate(pose);
	}

	public void setPositionXOffset(double x) {
		this.positionXOffset = x;
	}

	public void setPositionYOffset(double y) {
		this.positionYOffset = y;
	}

	public void addToPositionXOffset(double x) {
		this.positionXOffset += x;
	}

	public void addToPositionYOffset(double y) {
		this.positionYOffset += y;
	}

	public void setOffset(double x, double y) {
		this.positionXOffset = x;
		this.positionYOffset = y;
	}

	public void addToOffset(double x, double y) {
		this.addToPositionXOffset(x);
		this.addToPositionYOffset(y);
	}

	/**
	 * @return true if it is, false if it isn't
	 * @brief is the bot aiming at a point at the moment?
	 */
	public boolean isAiming() { return this.aimMode != AimMode.NONE; }

	public void aimAtPoint(double x, double y){
		this.aimAtPoint(x, y, 1.0);
	}

	/**
	 * @param x x value for point to aim at
	 * @brief set the bot to aim at a point, where the point is relative to the starting place of the robot
	 * <p>
	 * This overwrites any point that was there before
	 * @note uses RR axes (which are weird, look it up)
	 * @parma y y value for point to aim at
	 */
	public void aimAtPoint(double x, double y, double rotationSpeed) {
		this.aimMode = AimMode.POINT;

		this.aimAtPoint = new Vector2d(x, y);

		this.rotationSpeed = rotationSpeed;
	}

	public void aimAtAngleDegrees(double angle){
		this.aimAtAngleDegrees(angle, 1.0);
	}

	/**
	 * @param angle
	 * @brief aim at an angle, in degrees
	 */
	public void aimAtAngleDegrees(double angle, double rotationSpeed) {
		this.aimAtAngleRadians(Math.toRadians(angle), rotationSpeed);
	}

	/**
	 * @brief aim at angle radians, without a set rotation speed
	 *
	 * @param angle
	 */
	public void aimAtAngleRadians(double angle) {
		this.aimAtAngleRadians(angle, 1.0);
	}

	/**
	 * @brief aim at an angle, in radians
	 *
	 * @param angle
	 */
	public void aimAtAngleRadians(double angle, double rotationSpeed){
		this.aimMode = AimMode.ANGLE;

		this.aimAtAngle = angle;

		this.rotationSpeed = rotationSpeed;
	}

	public double getAimAtAngleRadians(){
		return this.aimAtAngle;
	}

	public double getAimAtAngleDegrees(){
		return Math.toDegrees(this.aimAtAngle);
	}

	public void aimAtCurrentAngle(){
		this.aimAtCurrentAngle(1.0);
	}

	/**
	 * @brief locks the drive's rotation onto its current heading until another call to an aimAt method or clearAim
	 */
	public void aimAtCurrentAngle(double rotationSpeed){
		// fetch angle from current pose
		double rotation = this.drive.getPoseEstimate().getHeading();

		this.aimAtAngleRadians(rotation, rotationSpeed);
	}

	/**
	 * @brief clears current aim, despite mode
	 */
	public void clearAim(){
		this.aimMode = AimMode.NONE;
	}

	public void setMode(DcMotor.RunMode mode){
		this.drive.setMode(mode);
	}

	private void setMotorVelocities(double frontLeft, double frontRight, double backLeft, double backRight){
		//this.drive.setMotorPowers(frontLeft, backLeft, backRight, frontRight);

		this.drive.leftFront.setVelocity(frontLeft);
		this.drive.leftRear.setVelocity(backLeft);
		this.drive.rightFront.setVelocity(frontRight);
		this.drive.rightRear.setVelocity(backRight);
	}

	private void setCumulativeVelocities(double frontLeft, double frontRight, double backLeft, double backRight){
		this.frontLeftVelocity = inchesToTicks(frontLeft);
		this.frontRightVelocity = inchesToTicks(frontRight);
		this.backLeftVelocity = inchesToTicks(backLeft);
		this.backRightVelocity = inchesToTicks(backRight);
	}

	private void addToCumulativeVelocities(double frontLeft, double frontRight, double backLeft, double backRight){
		this.frontLeftVelocity += inchesToTicks(frontLeft);
		this.frontRightVelocity += inchesToTicks(frontRight);
		this.backLeftVelocity += inchesToTicks(backLeft);
		this.backRightVelocity += inchesToTicks(backRight);
	}

	/**
	 * @brief Clears out the currently accumulated powers.  does not set the powers to the motor; that's what flushCumulativePowers is for
	 */
	private void clearCumulativeVelocities(){
		this.frontLeftVelocity = 0;
		this.frontRightVelocity = 0;
		this.backLeftVelocity = 0;
		this.backRightVelocity = 0;
	}

	/**
	 * @brief Flushes out the accumulated powers to the motors
	 */
	private void flushCumulativeVelocities(){
		this.setMotorVelocities(this.frontLeftVelocity, this.frontRightVelocity, this.backLeftVelocity, this.backRightVelocity);

		this.clearCumulativeVelocities();
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
		// technically doesn't work because the total velocity will exceed max velocity if forward and strafe are both equal to max velocity, but this will never matter when using direct controller inputs (which are already normalized)
		forward = Math.min(Math.max(forward, -this.maxVelocity), this.maxVelocity);
		strafe = Math.min(Math.max(strafe, -this.maxVelocity), this.maxVelocity);

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
		this.addToCumulativeVelocities(forward + strafe, forward - strafe, forward - strafe, forward + strafe);

		/*this.frontLeftVelocity += inchesToTicks(forward + strafe);
		this.frontRightVelocity += inchesToTicks(forward - strafe);
		this.backLeftVelocity += inchesToTicks(forward - strafe);
		this.backRightVelocity += inchesToTicks(forward + strafe);*/
	}

	/**
	 * This is a little unintuitive.  Best to just guess+check the best values here.
	 *
	 * @param rotation
	 */
	public void rotate(double rotation){
		this.addToCumulativeVelocities(rotation, -rotation, rotation, -rotation);

		/*this.frontLeftVelocity += inchesToTicks(rotation);
		this.frontRightVelocity -= inchesToTicks(rotation);
		this.backLeftVelocity += inchesToTicks(rotation);
		this.backRightVelocity -= inchesToTicks(rotation);*/
	}

	public void normalizePowers(){
		double largest = Math.max(this.backRightVelocity, Math.max(this.backLeftVelocity, Math.max(this.frontLeftVelocity, this.frontRightVelocity)));
		double maxVelocityTicks = inchesToTicks(this.maxVelocity); // TODO: just make a global?

		if(largest > maxVelocityTicks){
			this.frontLeftVelocity *= (maxVelocityTicks / largest);
			this.frontRightVelocity *= (maxVelocityTicks / largest);
			this.backLeftVelocity *= (maxVelocityTicks / largest);
			this.backRightVelocity *= (maxVelocityTicks / largest);
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
		double time = this.deltaTimer.seconds();
		double delta = time - this.lastTime;
		this.lastTime = time;

		if(this.isAiming() && !dontDoAim){
			// get current pose
			Pose2d pose = this.getPoseEstimate();
			Vector2d coords = pose.vec();
			double heading = pose.getHeading();

			if(this.aimMode == AimMode.POINT) {
				Vector2d offset = coords.minus(this.aimAtPoint);

				double distance = offset.norm();

				// if distance is less than a certain threshold, only update on an interval to avoid oscillation that happens at close range from the position (therefore the aimAtAngle) changing too rapidly
				if (distance != 0.0) {
					// get currently desired angle
					this.aimAtAngle = Math.atan2(offset.getY(), offset.getX()) + Math.asin(this.armOffset / distance);
					this.aimAtAngle += Math.PI;

					//this.lastAimUpdate = time;
				}
			}

			// pid
			// TODO: abstract into class/borrow a better implementation from somewhere

			// get error
			double error = -AngleHelper.angularDistanceRadians(heading, this.aimAtAngle);

			GlobalStorage.globalTelemetry.addData("error (radians)", error);
			GlobalStorage.globalTelemetry.addData("error (degrees)", Math.toDegrees(error));

			/*
			// get approximate rotational difference
			double distance = AbeConstants.DRIVE_DIAGONAL_RADIUS * error * 1.5;

			distance *= AbeConstants.DRIVE_AIM_RATE_OF_INCREASE;

			GlobalStorage.globalTelemetry.addData("distance", distance);

			if(!Double.isNaN(distance)) {
				// add rotation to cumulative powers
				this.addToCumulativeVelocities(distance, -distance, distance, -distance);
			}*/

			// get error derivative
			double derivative = (error - this.lastError) / delta;

			GlobalStorage.globalTelemetry.addData("derivative (radians)", derivative);

			// save error
			this.lastError = error;

			// add to integral
			if (!Double.isNaN(error)) this.totalError += error * delta;

			// calculate distance falloff
			double distanceFalloff = 1.0;

			if(this.aimMode == AimMode.POINT) {
				// FIXME: double calculation
				Vector2d offset = coords.minus(this.aimAtPoint);

				double distance = offset.norm();

				distanceFalloff = Math.min(1.0, distance / this.distanceFalloffRange);
			}

			// calculate rotation power
			double rotation = error * this.aim_p + derivative * this.aim_d + this.totalError * this.aim_i;

			// add a little bit to get us to the end (a little more reliable than the integral constant)
			// FIXME: not really customizable
			if(error < Math.toRadians(5.0) && error > Math.toRadians(1.0)){
				rotation += AbeConstants.DRIVE_EXTRA_KICK;
			}

			rotation *= distanceFalloff;
			rotation *= this.rotationSpeed;

			this.steady = Math.abs(error) < AbeDrive.STEADY_STATE_ERROR_TOLERANCE && Math.abs(derivative) < AbeDrive.STEADY_STATE_DERIVATIVE_TOLERANCE;

			// FIXME: remove this hack
			//double maxRotation = 0.5;
			//rotation = Math.max(Math.min(rotation, maxRotation), -maxRotation);

			if (!Double.isNaN(rotation)) {
				// add rotation to cumulative powers
				this.addToCumulativeVelocities(rotation, -rotation, rotation, -rotation);

				/*this.frontLeftVelocity += rotation;
				this.frontRightVelocity -= rotation;
				this.backLeftVelocity += rotation;
				this.backRightVelocity -= rotation;*/
			}
		} else {
			this.steady = true;
		}

		this.normalizePowers();

		// flush powers to motors
		this.flushCumulativeVelocities();
	}
}
