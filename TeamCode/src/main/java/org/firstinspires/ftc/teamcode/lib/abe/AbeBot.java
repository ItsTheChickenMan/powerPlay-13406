package org.firstinspires.ftc.teamcode.lib.abe;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.firstinspires.ftc.teamcode.lib.motion.AngleAdjuster;
import org.firstinspires.ftc.teamcode.lib.motion.LinearSlidesEx;
import org.firstinspires.ftc.teamcode.lib.motion.PositionableMotor;
import org.firstinspires.ftc.teamcode.lib.motion.PositionableServo;

/**
 * @brief Class for controlling an ABE bot (aka our power play bot v1)
 *
 * AbeBot is the only class that's allowed to rely on AbeConstants
 * @fixme I should follow the above rule more often
 */
public class AbeBot {
	public static class Hardware {
		/**
		 * @deprecated doesn't need to be assigned because of roadrunner.  not included in complete
		 */
		public DcMotorEx frontLeft;

		/**
		 * @deprecated doesn't need to be assigned because of roadrunner.  not included in complete
		 */
		public DcMotorEx frontRight;

		/**
		 * @deprecated doesn't need to be assigned because of roadrunner.  not included in complete
		 */
		public DcMotorEx backLeft;

		/**
		 * @deprecated doesn't need to be assigned because of roadrunner.  not included in complete
		 */
		public DcMotorEx backRight;

		public DcMotorEx elbow;
		public DcMotorEx slides;
		public DcMotorEx slides2;

		public Servo wristServo;
		public Servo fingerServo;

		public TouchSensor elbowLimitSensor;

		/**
		 * @deprecated doesn't need to be assigned because of roadrunner.  not included in complete
		 */
		public BNO055IMU imu;

		public void resetAllMotors(){
			/*
			this.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
			this.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
			this.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
			this.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
			*/

			this.elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
			this.slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
			this.slides2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		}

		public boolean complete(){
			// NOTE: I hate this method
			return 	/*this.frontLeft != null &&
							this.frontRight != null &&
							this.backLeft != null &&
							this.backRight != null &&*/
							this.elbow != null &&
							this.slides != null &&
							this.slides2 != null &&
							this.wristServo != null &&
							this.fingerServo != null &&
							this.elbowLimitSensor != null
							/*this.imu != null*/
							;
		}
	}

	// hardware
	private AbeBot.Hardware hardware;

	// drive train stuff
	//public MecanumDrivePlusOdo drive;
	public AbeDrive drive;

	// arm
	public AbeArm arm;

	// point being aimed at currently
	private Vector3D aimAtPoint;

	// activity of each aim component
	private boolean doDriveAim = true;
	private boolean doElbowAim = false;
	private boolean doSlidesAim = false;

	/**
	 * @brief Constructor
	 *
	 * @todo fix stupidity caused by roadrunner
	 *
	 * @throws Exception
	 */
	public AbeBot(HardwareMap hardwareMap) throws Exception {
		this.hardware = AbeConfig.loadHardware(hardwareMap);

		// check if hardware is complete
		if(!this.hardware.complete()){
			throw new Exception("some hardware missing");
		}

		// save hardware configuration
		this.hardware = hardware;

		// reset motors
		this.hardware.resetAllMotors();

		// create drive train
		// TODO: should we be using AbeConstants in here, or should that be left as a choice for the programmer?
		//this.drive = new MecanumDrivePlusOdo(this.hardware.frontLeft, this.hardware.frontRight, this.hardware.backLeft, this.hardware.backRight, AbeConstants.DRIVE_GEAR_RATIO, AbeConstants.DRIVE_TICK_RATIO, AbeConstants.DRIVE_WHEEL_RADIUS_INCHES, this.hardware.imu);
		this.drive = new AbeDrive(hardwareMap, AbeConstants.ARM_LATERAL_OFFSET_INCHES, AbeConstants.AIM_P_CONSTANT, AbeConstants.AIM_I_CONSTANT, AbeConstants.AIM_D_CONSTANT);

		this.arm = new AbeArm(
						new AngleAdjuster(this.hardware.elbow, AbeConstants.ELBOW_GEAR_RATIO, AbeConstants.ELBOW_TICK_RATIO, AbeConstants.ELBOW_LOWER_LIMIT_ROTATIONS, AbeConstants.ELBOW_UPPER_LIMIT_ROTATIONS, this.hardware.elbowLimitSensor),
						new LinearSlidesEx(
										new PositionableMotor[]{
														new PositionableMotor(this.hardware.slides, AbeConstants.SLIDE_GEAR_RATIO, AbeConstants.SLIDE_TICK_RATIO),
														new PositionableMotor(this.hardware.slides2, AbeConstants.SLIDE_GEAR_RATIO, AbeConstants.SLIDE_TICK_RATIO)
										},
										AbeConstants.SLIDE_BASE_LENGTH_INCHES, AbeConstants.SLIDE_BASE_LENGTH_INCHES + AbeConstants.SLIDE_MAX_EXTENSION_INCHES, AbeConstants.SLIDE_SPOOL_CIRCUMFERENCE_INCHES, AbeConstants.SLIDE_EXTENSION_FACTOR),
						new PositionableServo(this.hardware.wristServo, AbeConstants.WRIST_MAX_RANGE_RADIANS),
						new PositionableServo(this.hardware.fingerServo, AbeConstants.FINGERS_MAX_RANGE_RADIANS)
		);
	}

	public void setPoseEstimate(double x, double y, double r){
		this.drive.setPoseEstimate(x, y, r);
	}

	/**
	 * @brief Set the pose estimate from an existing pose
	 *
	 * It's only recommended to do this when taking a pose from auto, or something similar.  To provide custom starting values, use the other method since it uses "sane" coordinates
	 * @param pose
	 */
	public void setPoseEstimate(Pose2d pose){
		this.drive.setPoseEstimate(pose);
	}

	public void aimAt(double x, double y, double z){
		this.aimAt(x, y, z, true, true, true);
	}

	public void aimAt(double x, double y, double z, boolean doDrive, boolean doElbow, boolean doSlides){
		this.doDriveAim = doDrive;
		this.doElbowAim = doElbow;
		this.doSlidesAim = doSlides;

		this.drive.aimAtPoint(x, z);

		this.aimAtPoint = new Vector3D(x, y, z);
	}

	/**
	 * @brief make Abe aim himself at a point relative to the starting point with the arm and the drivetrain
	 *
	 * @param x
	 * @param y
	 * @param z
	 */
	public void aimAtPointFromStart(double x, double y, double z){
		this.aimAtPointFromStart(x, y, z, true, true, true);
	}

	public void aimAtPointFromStart(double x, double y, double z, boolean doDrive, boolean doElbow, boolean doSlides){
		this.doDriveAim = doDrive;
		this.doElbowAim = doElbow;
		this.doSlidesAim = doSlides;

		// get offset for drive
		Pose2d pose = this.drive.getPoseEstimate();

		// make drive aim at point
		this.drive.aimAtPoint(x - pose.getX(), z - pose.getY());

		// save point
		this.aimAtPoint = new Vector3D(x - pose.getX(), y, z - pose.getY());
	}

	public void clearPoint(){
		this.aimAtPoint = null;

		this.drive.clearAim();

		// TODO: why does this have different name
		this.arm.clearAim();
	}

	public boolean isAiming(){
		return this.aimAtPoint != null;
	}

	public void update(){
		// update drive
		this.drive.update(!doDriveAim);

		// update arm
		if(this.isAiming()) {
			Pose2d pose = this.drive.getPoseEstimate();

			double offsetX = pose.getX() - this.aimAtPoint.getX();
			double offsetZ = pose.getY() - this.aimAtPoint.getZ();

			// get arm values and stuff
			double botDistance2 = offsetX * offsetX + offsetZ * offsetZ;
			double armDistance = Math.sqrt(botDistance2 - AbeConstants.ARM_LATERAL_OFFSET_INCHES * AbeConstants.ARM_LATERAL_OFFSET_INCHES) - AbeConstants.ARM_LONGINAL_OFFSET_INCHES;
			double armHeight = this.aimAtPoint.getY() - AbeConstants.ARM_VERTICAL_OFFSET_INCHES;

			armDistance -= AbeConstants.WRIST_OFFSET_INCHES;

			//AimAtPointTest.globalTelemetry.addData("armDistance", armDistance);
			//AimAtPointTest.globalTelemetry.addData("armHeight", armHeight);

			this.arm.aimAt(armDistance, armHeight, this.doElbowAim, this.doSlidesAim);
		}

		this.arm.update();
	}
}
