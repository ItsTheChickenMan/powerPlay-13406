package org.firstinspires.ftc.teamcode.opmodes;

import android.icu.text.Transliterator;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.lib.abe.AbeConstants;
import org.firstinspires.ftc.teamcode.lib.motion.PositionableMotor;

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

		PositionableMotor frontLeftP = new PositionableMotor(frontLeft, AbeConstants.DRIVE_GEAR_RATIO, AbeConstants.DRIVE_TICK_RATIO);
		PositionableMotor frontRightP = new PositionableMotor(frontRight, AbeConstants.DRIVE_GEAR_RATIO, AbeConstants.DRIVE_TICK_RATIO);
		PositionableMotor backLeftP = new PositionableMotor(backLeft, AbeConstants.DRIVE_GEAR_RATIO, AbeConstants.DRIVE_TICK_RATIO);
		PositionableMotor backRightP = new PositionableMotor(backRight, AbeConstants.DRIVE_GEAR_RATIO, AbeConstants.DRIVE_TICK_RATIO);

		waitForStart();

		while(opModeIsActive()){
			frontLeftP.setPower(1.0);
			frontRightP.setPower(1.0);
			backLeftP.setPower(1.0);
			backRightP.setPower(1.0);
		}
	}
}
