package org.firstinspires.ftc.teamcode.lib.drive;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.utils.Imu;
import org.firstinspires.ftc.teamcode.lib.utils.Vec2;

/**
 * @brief This doesn't use PositionableMotor so it's fairly unreliable if the encoders are reset.
 */
public class MecanumDrivePlusOdo extends MecanumDrive {
	private Vec2 position;

	private Imu imu;

	private double wheelRadius;
	private double wheelCircumference;

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
	public MecanumDrivePlusOdo(DcMotorEx frontLeft, DcMotorEx frontRight, DcMotorEx backLeft, DcMotorEx backRight, double gearRatio, double tickRatio, double wheelRadius, BNO055IMU imu) {
		super(frontLeft, frontRight, backLeft, backRight, gearRatio, tickRatio);

		this.position = new Vec2();
		this.wheelRadius = wheelRadius;
		this.wheelCircumference = 2 * Math.PI * this.wheelRadius;
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
	public MecanumDrivePlusOdo(DcMotorEx frontLeft, DcMotorEx frontRight, DcMotorEx backLeft, DcMotorEx backRight, double gearRatio, double tickRatio, double wheelRadius, Imu imu) {
		super(frontLeft, frontRight, backLeft, backRight, gearRatio, tickRatio);

		this.position = new Vec2();
		this.wheelRadius = wheelRadius;
		this.wheelCircumference = 2 * Math.PI * this.wheelRadius;
		this.imu = imu;
	}

	public double getX(){
		return this.position.x;
	}

	public double getY(){
		return this.position.y;
	}

	public double getAngleDegrees(){
		return this.imu.getAngleDegrees();
	}

	public double getAngleRadians(){
		return this.imu.getAngleRadians();
	}

	public void update(double delta, Telemetry telemetry) {
		// calculate velocity vector from encoder velocities and current imu orientation
		double fl = this.frontLeft.getVelocityRadians();
		double fr = this.frontRight.getVelocityRadians();
		double bl = this.backLeft.getVelocityRadians();
		double br = this.backRight.getVelocityRadians();

		//telemetry.addData("delta", delta);

		fl *= delta;
		fr *= delta;
		bl *= delta;
		br *= delta;

		// FIXME: use actual vectors
		double x = 0;
		double y = 0;

		// TODO: abstract into method

		// wheel contributions
		double frontLeftContribution = fl * this.wheelCircumference;
		double frontRightContribution = fr * this.wheelCircumference;
		double backLeftContribution = bl * this.wheelCircumference;
		double backRightContribution = br * this.wheelCircumference;

		telemetry.addData("frontRightRaw", this.frontRight.getRawVelocity());
		telemetry.addData("frontRightRotations", this.frontRight.getVelocityRotations());
		telemetry.addData("velocityRadians", this.frontRight.getVelocityRadians());
		telemetry.addData("velocityDeltaCorrected", fr);
		telemetry.addData("frontRightContribution", frontRightContribution);

		// front left
		x -= frontLeftContribution;
		y += frontLeftContribution;

		// front right
		x += frontRightContribution;
		y += frontRightContribution;

		// back left
		x += backLeftContribution;
		y += backLeftContribution;

		// back right
		x -= backRightContribution;
		y += backRightContribution;

		telemetry.addData("yFull", y);

		// average
		x /= 4.0;
		y /= 4.0;

		telemetry.addData("yAvg", y);

		telemetry.update();

		// rotate (this will create a disparity where the update rate will affect odometry accuracy while rotating)
		Vec2 velocity = new Vec2(x, y);
		//velocity.rotate(this.imu.getAngleRadians());

		this.position.add(velocity);
	}
}
