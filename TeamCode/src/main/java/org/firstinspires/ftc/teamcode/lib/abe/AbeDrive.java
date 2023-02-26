package org.firstinspires.ftc.teamcode.lib.abe;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.lib.utils.GlobalStorage;

public class AbeDrive {
	public enum AimMode {
		NONE,
		POINT,
		ANGLE
	}

	// roadrunner drive
	private SampleMecanumDrive roadrunnerDrive;
	private Vector2d poseOffset; // repeatedly calling setPoseEstimate seems to break roadrunner, so we use this instead

	// drive details... //
	private Pose2d desiredMovement;

	// aim details... //
	private PIDFController headingController;
	private AimMode currentAimMode;
	private Vector2d aimPoint;

	// steady state
	private double lastErrorRadians = 0.0;
	private double lastDerivativeRadians = 0.0;

	private double steadyStateMaximumErrorRadians= Math.toRadians(3.0);
	private double steadyStateMaximumDerivativeRadians = Math.toRadians(2.0);

	private ElapsedTime deltaTimer;

	public AbeDrive(HardwareMap hardwareMap, SampleMecanumDrive.LocalizationType localizationType){
		this.roadrunnerDrive = new SampleMecanumDrive(hardwareMap, localizationType);
		this.headingController = new PIDFController(AbeConstants.AIM_HEADING_PID);

		this.headingController.setInputBounds(-Math.PI, Math.PI);

		this.poseOffset = new Vector2d(0, 0);
		this.desiredMovement = new Pose2d(0, 0, 0);
		this.currentAimMode = AimMode.NONE;
		this.aimPoint = null;

		this.deltaTimer = new ElapsedTime();
	}

	/**
	 * @brief check if drive is steady using custom tolerances
	 *
	 * @param maxErrorRadians
	 * @param maxDerivativeRadians
	 * @return true if drive is steady (within a certain error and speed threshold)
	 */
	public boolean isSteady(double maxErrorRadians, double maxDerivativeRadians){
		return (this.lastErrorRadians < maxErrorRadians) && (this.lastDerivativeRadians < maxDerivativeRadians);
	}

	/**
	 * @brief check if drive is steady using custom tolerances
	 *
	 * @return true if drive is steady (within a certain error and speed threshold)
	 */
	public boolean isSteady(){
		GlobalStorage.globalTelemetry.addData("last error radians", this.lastErrorRadians);
		GlobalStorage.globalTelemetry.addData("last derivative radians", this.lastDerivativeRadians);

		return isSteady(steadyStateMaximumErrorRadians, steadyStateMaximumDerivativeRadians);
	}

	public void setDefaultSteadyStateMaximumErrorRadians(double radians){
		this.steadyStateMaximumErrorRadians = radians;
	}

	public void setDefaultSteadyStateMaximumDerivativeRadians(double radians){
		this.steadyStateMaximumDerivativeRadians = radians;
	}
	/**
	 * @param startPose
	 * @return passthrough to rr drive trajectory builder
	 */
	public TrajectoryBuilder trajectoryBuilder(Pose2d startPose){
		return this.roadrunnerDrive.trajectoryBuilder(startPose);
	}

	/**
	 * @brief follow a trajectory synchronously, blocking until complete
	 *
	 * @param trajectory
	 */
	public void followTrajectory(Trajectory trajectory){
		this.roadrunnerDrive.followTrajectory(trajectory);
	}

	/**
	 * @return see SampleMecanumDrive getRawExternalHeading
	 */
	public double getRawExternalHeading(){
		return this.roadrunnerDrive.getRawExternalHeading();
	}

	/**
	 * @brief follow a trajectory asynchronously, non-blocking but need to update drive
	 *
	 * @param trajectory
	 */
	public void followTrajectoryAsync(Trajectory trajectory){
		this.roadrunnerDrive.followTrajectoryAsync(trajectory);
	}

	/**
	 * @return the direction that the drive is facing as a vector
	 */
	public Vector2d getForwardVector(){
		return new Vector2d(1, 0).rotated(this.getPoseEstimate().getHeading());
	}

	/**
	 * @return roadrunner drive pose estimate, corrected using the pose offset
	 */
	public Pose2d getPoseEstimate(){
		Pose2d corrected = new Pose2d(this.roadrunnerDrive.getPoseEstimate().vec().plus(this.poseOffset), this.roadrunnerDrive.getPoseEstimate().getHeading());

		return corrected;
	}

	/**
	 * @brief sets the roadrunner pose estimate to some value and resets the pose offset
	 *
	 * @param pose
	 */
	public void setPoseEstimate(Pose2d pose){
		this.roadrunnerDrive.setPoseEstimate(pose);

		this.setPoseOffset(new Vector2d(0, 0));
	}

	public void setPoseOffset(Vector2d offset){
		this.poseOffset.copy(offset.getX(), offset.getY());
	}

	public void addToPoseOffset(Vector2d offset){
		this.poseOffset = this.poseOffset.plus(offset);
	}

	/**
	 * @brief calculate the rotation of the drive train required to aim at a point relative to its center.
	 *
	 * @param offset point relative to the robot's center
	 */
	public double calculateAimAngle(Vector2d offset){
		return -(Math.atan2(-offset.getY(), offset.getX()) - Math.asin(-AbeConstants.ARM_Z_OFFSET_INCHES / offset.norm()));
	}

	/**
	 * @brief instruct the drive train to aim at a particular point, relative to the bottom right corner of the field
	 *
	 * @param x
	 * @param y
	 */
	public void aimAtPoint(double x, double y){
		this.aimPoint = new Vector2d(x, y);
		this.currentAimMode = AimMode.POINT;
	}

	/**
	 * @brief instruct the drive train to hold a particular angle
	 *
	 * @param theta
	 */
	public void aimAtAngleRadians(double theta){
		this.aimPoint = new Vector2d(theta, theta);
		this.currentAimMode = AimMode.ANGLE;
	}

	public void clearAim(){
		this.currentAimMode = AimMode.NONE;
	}

	public boolean isAiming(){
		return this.aimPoint != null;
	}

	public double getAimErrorRadians(){
		if(currentAimMode != AimMode.NONE){
			return this.headingController.getLastError();
		} else {
			return 0.0;
		}
	}

	public double getAimErrorDegrees(){
		return Math.toDegrees(getAimErrorRadians());
	}

	public void updateSteadyState(){
		double dt = this.deltaTimer.seconds();
		this.deltaTimer.reset();

		double error = this.getAimErrorRadians();
		double derivative = (error - this.lastErrorRadians) / dt;

		this.lastErrorRadians = error;
		this.lastDerivativeRadians = derivative;
	}

	/**
	 * @brief instruct the drive train to drive using field oriented with these values on the next update
	 *
	 * r is ignored if aim mode is anything except for NONE
	 *
	 * @param x desired forward movement
	 * @param y desired side movement
	 * @param r desired rotation (ignored if aim mode isn't NONE)
	 */
	public void driveFieldOriented(double x, double y, double r){
		this.desiredMovement = new Pose2d(x, y, r);
	}

	public void update(){
		update(false);
	}

	public void update(boolean useThetaFF){
		// get pose estimate
		Pose2d poseEstimate = this.roadrunnerDrive.getPoseEstimate();

		// calculate actual desired movement
		Vector2d driveDirection = this.desiredMovement.vec().rotated(-poseEstimate.getHeading());

		double thetaFF = 0.0;

		// update movement for aim mode
		switch(this.currentAimMode){
			case POINT: {
				// break if point is null
				if(!this.isAiming()) break;

				Vector2d offset = this.aimPoint.minus(this.getPoseEstimate().vec());
				double distance = offset.norm();

				// distances of 0 screw it up
				if(distance > 1) {
					// calculate angle to point
					double theta = this.calculateAimAngle(this.aimPoint.minus(this.getPoseEstimate().vec()));

					Pose2d velocity = this.roadrunnerDrive.getPoseVelocity();

					// stole this from noah bres
					if(velocity != null && useThetaFF) thetaFF = -velocity.vec().rotated(-Math.PI / 2).dot(offset) / (distance * distance);

					// update pidf controller
					headingController.setTargetPosition(theta);
				}

				break;
			}

			case ANGLE: {
				// break if angle is null
				// (uses the first component of aimPoint)
				if(!this.isAiming()) break;

				// get angle
				double theta = this.aimPoint.getX();

				// update pidf controller
				headingController.setTargetPosition(theta);

				break;
			}
		}

		// calculate heading input
		double headingInput = this.desiredMovement.getHeading();

		if(this.currentAimMode != AimMode.NONE){
			// calculate power
			// TODO: add in feedforward?
			headingInput = (headingController.update(poseEstimate.getHeading()) * DriveConstants.kV + thetaFF) * DriveConstants.TRACK_WIDTH;
		}

		// cap heading input
		headingInput = Math.min(Math.max(headingInput, -AbeConstants.MAX_ROTATION_POWER), AbeConstants.MAX_ROTATION_POWER);

		// calculate drive movement
		Pose2d driveMovement = new Pose2d(driveDirection, headingInput);

		// set weighted drive powers
		roadrunnerDrive.setWeightedDrivePower(driveMovement);

		// update pidf controller
		headingController.update(poseEstimate.getHeading());

		// update steady state stuff
		this.updateSteadyState();

		this.updateRoadrunnerDrive();

		// clear desired movement
		this.desiredMovement = new Pose2d();
	}

	public void updateRoadrunnerDrive(){
		// update drive
		this.roadrunnerDrive.update();
	}
}
