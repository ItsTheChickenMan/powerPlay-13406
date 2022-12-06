package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

// back right motor started giving me some problems.  this is to test it
// NOTE: it was just an unplugged encoder cable, lol
@TeleOp
public class BackRightMotorTest extends LinearOpMode {
	@Override
	public void runOpMode() throws InterruptedException {
		// load backright
		DcMotorEx backRight = hardwareMap.get(DcMotorEx.class, "backRight");

		backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

		waitForStart();

		backRight.setVelocity(500);

		while(opModeIsActive()){
			telemetry.addData("position", backRight.getCurrentPosition());
			telemetry.update();
		}
	}
}
