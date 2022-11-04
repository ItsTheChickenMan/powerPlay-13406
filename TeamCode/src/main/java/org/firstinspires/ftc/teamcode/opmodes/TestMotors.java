package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class TestMotors extends LinearOpMode {

	@Override
	public void runOpMode() throws InterruptedException {
		DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
		DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
		DcMotorEx backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
		DcMotorEx backRight = hardwareMap.get(DcMotorEx.class, "backRight");

		frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		waitForStart();

		while(opModeIsActive()){
			frontLeft.setPower(1.0);
			frontRight.setPower(1.0);
			backLeft.setPower(1.0);
			backRight.setPower(1.0);
		}
	}
}
