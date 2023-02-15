package org.firstinspires.ftc.teamcode.lib.abe;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.Vector;
import java.util.regex.PatternSyntaxException;

public class AbeDrive {
	public enum AimMode {
		NONE,
		POINT
	}

	// roadrunner drive
	private SampleMecanumDrive roadrunnerDrive;

	// drive details... //
	private Pose2d desiredMovement;

	// aim details... //
	private PIDFController headingController;
	private AimMode currentAimMode;
	private Vector2d aimPoint;

	public AbeDrive(HardwareMap hardwareMap){
		this.roadrunnerDrive = new SampleMecanumDrive(hardwareMap);
		this.headingController = new PIDFController(SampleMecanumDrive.HEADING_PID);
	}

	/**
	 * @return roadrunner drive pose estimate
	 */
	public Pose2d getPoseEstimate(){
		return this.roadrunnerDrive.getPoseEstimate();
	}

	/**
	 * @brief calculate the rotation of the drive train required to aim at a point relative to its center.
	 *
	 * @param offset point relative to the robot's center
	 */
	public double calculateAimAngle(Vector2d offset){
		double angle = Math.atan2(-offset.getY(), offset.getX()) - Math.asin(-AbeConstants.ARM_Y_OFFSET_INCHES / offset.norm());

		return angle;
	}

	public void aimAt(double x, double y){
		this.aimPoint = new Vector2d(x, y);
		this.currentAimMode = AimMode.POINT;
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

		Pose2d driveMovement = new Pose2d(driveDirection, this.desiredMovement.getHeading());

		// update movement for aim mode
		switch(this.currentAimMode){
			case POINT: {
				// break if point is null
				if(this.aimPoint != null) break;

				// calculate angle to point
				double theta = this.calculateAimAngle(this.aimPoint);

				// update pidf controller
				headingController.setTargetPosition(theta);

				// calculate power
				// TODO: add in feedforward?
				double headingInput = (headingController.update(poseEstimate.getHeading()) * DriveConstants.kV/* + thetaFF */) * DriveConstants.TRACK_WIDTH;

				// set heading input
				driveMovement = new Pose2d(driveDirection, headingInput);
			}
		}

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
