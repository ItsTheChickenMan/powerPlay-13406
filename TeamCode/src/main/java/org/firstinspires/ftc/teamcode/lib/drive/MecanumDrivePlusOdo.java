package org.firstinspires.ftc.teamcode.lib.drive;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.vuforia.Vec2F;
import com.vuforia.Vec3F;

import org.firstinspires.ftc.teamcode.lib.utils.Imu;

/**
 * @brief This doesn't use PositionableMotor so it's fairly unreliable if the encoders are reset.
 */
public class MecanumDrivePlusOdo extends MecanumDrive {
	private Vec2F position;

	private Imu imu;

	/**
	 * @brief Construct a mecanum drive from four initialized motors
	 *
	 * @note THIS DOES NOT SET DIRECTIONS FOR YOU!  Set them yourself.
	 *
	 * @param frontLeft  front left motor
	 * @param frontRight front right motor
	 * @param backLeft back left motor
	 * @param backRight back right motor
	 */
	public MecanumDrivePlusOdo(DcMotorEx frontLeft, DcMotorEx frontRight, DcMotorEx backLeft, DcMotorEx backRight, BNO055IMU imu) {
		super(frontLeft, frontRight, backLeft, backRight);

		this.imu = new Imu(imu);
	}

	/**
	 * @brief Construct a mecanum drive from four initialized motors
	 *
	 * @note THIS DOES NOT SET DIRECTIONS FOR YOU!  Set them yourself.
	 *
	 * @param frontLeft  front left motor
	 * @param frontRight front right motor
	 * @param backLeft back left motor
	 * @param backRight back right motor
	 */
	public MecanumDrivePlusOdo(DcMotorEx frontLeft, DcMotorEx frontRight, DcMotorEx backLeft, DcMotorEx backRight, Imu imu) {
		super(frontLeft, frontRight, backLeft, backRight);

		this.imu = imu;
	}

	public void update() {

	}
}
