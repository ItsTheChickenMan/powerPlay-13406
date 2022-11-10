package org.firstinspires.ftc.teamcode.lib.abe;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.lib.drive.MecanumDrivePlusOdo;
import org.firstinspires.ftc.teamcode.lib.motion.AngleAdjuster;
import org.firstinspires.ftc.teamcode.lib.motion.LinearSlides;
import org.firstinspires.ftc.teamcode.lib.motion.PositionableMotor;
import org.firstinspires.ftc.teamcode.lib.motion.PositionableServo;

/**
 * @brief Class for controlling an ABE bot (aka our power play bot v1)
 *
 */
public class AbeBot {
	public static class Hardware {
		public DcMotorEx frontLeft;
		public DcMotorEx frontRight;
		public DcMotorEx backLeft;
		public DcMotorEx backRight;

		public DcMotorEx elbow;
		public DcMotorEx slides;

		public Servo wristServo;
		public Servo fingerServo;

		public TouchSensor elbowLimitSensor;

		public BNO055IMU imu;

		public void resetAllMotors(){
			this.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
			this.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
			this.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
			this.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

			this.elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
			this.slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		}

		public boolean complete(){
			// NOTE: I hate this method
			return 	this.frontLeft != null &&
							this.frontRight != null &&
							this.backLeft != null &&
							this.backRight != null &&
							this.elbow != null &&
							this.slides != null &&
							this.wristServo != null &&
							this.fingerServo != null &&
							this.elbowLimitSensor != null &&
							this.imu != null;
		}
	}

	// hardware
	private AbeBot.Hardware hardware;

	// drive train stuff
	public MecanumDrivePlusOdo drive;

	// arm
	public Arm arm;

	/**
	 * @brief Constructor
	 *
	 * @param hardware All hardware needed by AbeBot.  If incomplete, throws exception.
	 * @throws Exception
	 */
	public AbeBot(@NonNull AbeBot.Hardware hardware) throws Exception {
		// check if hardware is complete
		if(!hardware.complete()){
			throw new Exception("some hardware missing");
		}

		// save hardware configuration
		this.hardware = hardware;

		// reset motors
		this.hardware.resetAllMotors();

		// create drive train
		// TODO: should we be using AbeConstants in here, or should that be left as a choice for the programmer?
		this.drive = new MecanumDrivePlusOdo(this.hardware.frontLeft, this.hardware.frontRight, this.hardware.backLeft, this.hardware.backRight, AbeConstants.DRIVE_GEAR_RATIO, AbeConstants.DRIVE_TICK_RATIO, AbeConstants.DRIVE_WHEEL_RADIUS_INCHES, this.hardware.imu);

		this.arm = new Arm(
						new AngleAdjuster(this.hardware.elbow, AbeConstants.ELBOW_GEAR_RATIO, AbeConstants.ELBOW_TICK_RATIO, this.hardware.elbowLimitSensor),
						new LinearSlides(new PositionableMotor(this.hardware.slides, AbeConstants.SLIDE_GEAR_RATIO, AbeConstants.SLIDE_TICK_RATIO), AbeConstants.SLIDE_BASE_LENGTH_INCHES, AbeConstants.SLIDE_BASE_LENGTH_INCHES + AbeConstants.SLIDE_MAX_EXTENSION_INCHES, AbeConstants.SLIDE_SPOOL_RADIUS_INCHES),
						new PositionableServo(this.hardware.wristServo, AbeConstants.WRIST_MAX_RANGE_RADIANS),
						new PositionableServo(this.hardware.fingerServo, AbeConstants.FINGERS_MAX_RANGE_RADIANS)
		);
	}
}
