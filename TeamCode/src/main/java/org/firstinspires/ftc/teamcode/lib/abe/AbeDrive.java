package org.firstinspires.ftc.teamcode.lib.abe;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

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
	private double steadyStateMaximumErrorRadians;
	private double steadyStateMaximumDerivativeRadians;

	public AbeDrive(HardwareMap hardwareMap, SampleMecanumDrive.LocalizationType localizationType){
		this.roadrunnerDrive = new SampleMecanumDrive(hardwareMap, localizationType);
		this.headingController = new PIDFController(AbeConstants.AIM_HEADING_PID);

		this.headingController.setInputBounds(-Math.PI, Math.PI);

		this.poseOffset = new Vector2d(0, 0);
		this.desiredMovement = new Pose2d(0, 0, 0);
		this.currentAimMode = AimMode.NONE;
		this.aimPoint = null;
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
		return Math.atan2(-offset.getY(), offset.getX()) - Math.asin(-AbeConstants.ARM_Z_OFFSET_INCHES / offset.norm());
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
		// get pose estimate
		Pose2d poseEstimate = this.roadrunnerDrive.getPoseEstimate();

		// calculate actual desired movement
		Vector2d driveDirection = this.desiredMovement.vec().rotated(-poseEstimate.getHeading());

		// update movement for aim mode
		switch(this.currentAimMode){
			case POINT: {
				// break if point is null
				if(!this.isAiming()) break;

				// calculate angle to point
				double theta = this.calculateAimAngle(this.aimPoint);

				// update pidf controller
				headingController.setTargetPosition(theta);

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
			headingInput = (headingController.update(poseEstimate.getHeading()) * DriveConstants.kV/* + thetaFF */) * DriveConstants.TRACK_WIDTH;
		}

		// calculate drive movement
		Pose2d driveMovement = new Pose2d(driveDirection, headingInput);

		// set weighted drive powers
		roadrunnerDrive.setWeightedDrivePower(driveMovement);

		// update pidf controller
		headingController.update(poseEstimate.getHeading());

		// update pose estimate
		this.roadrunnerDrive.update();

		// clear desired movement
		this.desiredMovement = new Pose2d();
	}
}
