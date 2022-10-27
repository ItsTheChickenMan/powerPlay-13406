package org.firstinspires.ftc.teamcode.lib.Abe;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.lib.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.lib.drive.MecanumDrivePlusOdo;
import org.firstinspires.ftc.teamcode.lib.utils.Imu;

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

		public DcMotorEx angleAdjusterMotor;
		public DcMotorEx slidesMotor;

		public Servo wristServo;
		public Servo fingerServo;

		public BNO055IMU imu;

		public boolean complete(){
			// NOTE: I hate this method
			return 	this.frontLeft != null &&
							this.frontRight != null &&
							this.backLeft != null &&
							this.backRight != null &&
							this.angleAdjusterMotor != null &&
							this.slidesMotor != null &&
							this.wristServo != null &&
							this.fingerServo != null &&
							this.imu != null;
		}
	}

	// hardware
	private AbeBot.Hardware hardware;

	// drive train stuff
	private MecanumDrivePlusOdo drive;

	// arm
	private Arm arm;

	public AbeBot(@NonNull AbeBot.Hardware hardware) throws Exception {
		// check if hardware is complete
		if(!hardware.complete()){
			throw new Exception("some hardware missing");
		}

		// save hardware configuration
		this.hardware = hardware;

		// create drive train
		this.drive = new MecanumDrivePlusOdo(this.hardware.frontLeft, this.hardware.frontRight, this.hardware.backLeft, this.hardware.backRight, this.hardware.imu);
	}
}
