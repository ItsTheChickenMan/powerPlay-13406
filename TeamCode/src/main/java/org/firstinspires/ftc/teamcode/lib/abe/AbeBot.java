package org.firstinspires.ftc.teamcode.lib.abe;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.lib.motion.LinearSlides;
import org.firstinspires.ftc.teamcode.lib.motion.PositionableMotor;
import org.firstinspires.ftc.teamcode.lib.motion.PositionableServo;

public class AbeBot {
	// hardware... //

	// drive
	public final AbeDrive drive;

	// arm
	public final AbeArm arm;

	// aim details...//
	private Vector3D aimAtPoint; // generally we use roadrunner vectors, but there's no Vector3d for roadrunner

	public AbeBot(HardwareMap hardwareMap, SampleMecanumDrive.LocalizationType localizationType, PIDCoefficients aimHeadingCoefficients){
		// get hardware
		AbeConfig.Hardware hardware = AbeConfig.loadHardware(hardwareMap);

		// construct drive
		this.drive = new AbeDrive(hardwareMap, localizationType, aimHeadingCoefficients);

		// load motors
		PositionableMotor elbowMotor = new PositionableMotor(hardware.elbowMotor, AbeConstants.ELBOW_GEAR_RATIO, AbeConstants.ELBOW_TPR, AbeConstants.ELBOW_LOWER_LIMIT_RADIANS, AbeConstants.ELBOW_UPPER_LIMIT_RADIANS);
		PositionableMotor slidesMotor = new PositionableMotor(hardware.slidesMotor, AbeConstants.SLIDES_GEAR_RATIO, AbeConstants.SLIDES_TPR);

		// load servo
		PositionableServo wristServo = new PositionableServo(hardware.wristServo, Math.toRadians(AbeConstants.WRIST_MAX_RANGE_DEGREES), Math.toRadians(AbeConstants.WRIST_ZERO_ANGLE_DEGREES));
		PositionableServo clawServo = new PositionableServo(hardware.clawServo, Math.toRadians(AbeConstants.CLAW_MAX_RANGE_DEGREES), Math.toRadians(AbeConstants.CLAW_ZERO_ANGLE_DEGREES));

		// construct slides
		// NOTE: not using extension factor because it was a band-aid fix for a stupid bug that's since been fixed for real.  plus it could just be factored into the circumference and have the exact same effect
		LinearSlides slides = new LinearSlides(slidesMotor, AbeConstants.SLIDES_BASE_LENGTH_INCHES, AbeConstants.SLIDES_BASE_LENGTH_INCHES + AbeConstants.SLIDES_MAX_EXTENSION_INCHES, AbeConstants.SLIDES_SPOOL_CIRCUMFERENCE_INCHES, 1.0);

		// construct hand
		AbeHand hand = new AbeHand(wristServo, clawServo);

		// construct arm
		this.arm = new AbeArm(elbowMotor, slides, hand, AbeConstants.ARM_TUNING_POINTS);
	}

	public boolean isSteady(){
		return this.arm.isSteady() && this.drive.isSteady();
	}

	/**
	 * @brief Bring the end of the claw to some point (x, y, z) in space (relative to bottom right corner, not robot)
	 *
	 * Unlike the arm, update must be called repeatedly for this to work properly (to update odo)
	 *
	 * @param x
	 * @param y
	 * @param z
	 */
	public void aimAt(double x, double y, double z){
		// save point
		this.aimAtPoint = new Vector3D(x, y, z);

		// instruct drive to aim (only needs to be told once)
		this.drive.aimAtPoint(this.aimAtPoint.getX(), this.aimAtPoint.getZ());
	}

	/**
	 * @brief Tell the robot to stop aiming
	 */
	public void clearAim(){
		this.aimAtPoint = null;

		// clear drive aim
		this.drive.clearAim();
	}

	/**
	 * @return is abe aiming at a point currently?
	 */
	public boolean isAiming(){
		return this.aimAtPoint != null;
	}

	private double calculateArmDistance(Vector3D start, Vector3D end){
		double offsetX = end.getX() - start.getX();
		double offsetY = end.getZ() - start.getZ();

		double botDistance2 = offsetX*offsetX + offsetY*offsetY;
		double armDistance = Math.sqrt(botDistance2 - AbeConstants.ARM_Y_OFFSET_INCHES * AbeConstants.ARM_Y_OFFSET_INCHES) - AbeConstants.ARM_X_OFFSET_INCHES;

		// NOTE: now taken care of by arm class
		// armDistance -= AbeConstants.WRIST_OFFSET_INCHES;

		return armDistance;
	}

	private double calculateArmHeight(Vector3D start, Vector3D end){
		return end.getY() - AbeConstants.ARM_Y_OFFSET_INCHES;
	}

	public void updateArm(){
		this.updateArm(true, true);
	}

	public void updateArm(boolean doElbow, boolean doSlides){
		// do aim logic
		if(this.isAiming()){
			Pose2d poseEstimate = this.drive.getPoseEstimate();

			Vector3D poseEstimateVec3 = new Vector3D(poseEstimate.getX(), 0, poseEstimate.getY());

			double armDistance = this.calculateArmDistance(poseEstimateVec3, this.aimAtPoint);
			double armHeight = this.calculateArmHeight(poseEstimateVec3, this.aimAtPoint);

			// aim
			this.arm.aimAt(armDistance, armHeight);
		}

		// update arm
		this.arm.update(doElbow, doSlides);
	}

	public void updateDrive(){
		this.drive.update();
	}

	public void updateDriveNoRoadrunner(){
		this.drive.updateNoRoadrunnerDrive();
	}

	/**
	 * @brief update the robot parts specified, and do aim logic if needed
	 *
	 * @param doDrive do drive train update
	 * @param doElbow move the elbow
	 * @param doSlides move the slides
	 */
	public void updateNoRoadrunner(boolean doDrive, boolean doElbow, boolean doSlides){
		// update drive train
		if(doDrive){
			this.updateDriveNoRoadrunner();
		}

		this.updateArm(doElbow, doSlides);
	}

	/**
	 * @brief update all of the robot parts, do aim logic if needed
	 */
	public void updateNoRoadrunner(){
		this.update(true, true, true);
	}

	public void update(boolean doDrive, boolean doElbow, boolean doSlides){
		if(doDrive){
			this.updateDrive();
		}

		this.updateArm(doElbow, doSlides);
	}

	public void update(){
		this.update(true, true, true);
	}
}
