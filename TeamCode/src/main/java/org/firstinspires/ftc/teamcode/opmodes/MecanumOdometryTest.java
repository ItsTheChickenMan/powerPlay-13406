package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.abe.AbeConstants;
import org.firstinspires.ftc.teamcode.lib.drive.MecanumDrivePlusOdo;
import org.firstinspires.ftc.teamcode.lib.utils.Imu;

@TeleOp(group = "Tests")
public class MecanumOdometryTest extends LinearOpMode {
	private MecanumDrivePlusOdo drivePlusOdo;

	@Override
	public void runOpMode() throws InterruptedException {
		DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
		DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
		DcMotorEx backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
		DcMotorEx backRight = hardwareMap.get(DcMotorEx.class, "backRight");
		BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");

		frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
		backRight.setDirection(DcMotorSimple.Direction.REVERSE);

		drivePlusOdo = new MecanumDrivePlusOdo(frontLeft, frontRight, backLeft, backRight, AbeConstants.DRIVE_GEAR_RATIO, AbeConstants.DRIVE_TICK_RATIO, AbeConstants.DRIVE_WHEEL_CIRCUMFERENCE_INCHES, new Imu(imu));

		waitForStart();

		ElapsedTime time = new ElapsedTime();

		double delta = 0;
		double lastTime = time.seconds();

		while(opModeIsActive()) {
			/*telemetry.addData("x", this.drivePlusOdo.getX());
			telemetry.addData("y", this.drivePlusOdo.getY());
			telemetry.update();*/


			drivePlusOdo.driveNormal(-gamepad1.left_stick_y, 0 /*gamepad1.left_stick_x*/, gamepad1.right_stick_x, 1.3 - gamepad1.left_trigger);

			double t = time.seconds();
			delta = t - lastTime;
			lastTime = t;

			drivePlusOdo.update(delta, telemetry);
		}
	}
}
