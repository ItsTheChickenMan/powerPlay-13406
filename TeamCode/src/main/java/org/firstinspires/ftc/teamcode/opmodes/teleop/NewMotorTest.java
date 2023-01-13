package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.lib.motion.PositionableMotor;

@TeleOp(group = "Tests")
public class NewMotorTest extends LinearOpMode {
	@Override
	public void runOpMode() throws InterruptedException {
		DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "parallelEncoder");

		PositionableMotor pMotor = new PositionableMotor(motor, 1, 1425.1);

		waitForStart();

		pMotor.rotateToDegrees(0.0, 25.0);

		while(!gamepad2.a && !isStopRequested());

		pMotor.rotateToDegrees(45.0, 25.0);

		sleep(500);
		while(!gamepad2.a && !isStopRequested());

		pMotor.rotateToDegrees(90.0, 25.0);

		sleep(500);
		while(!gamepad2.a && !isStopRequested());

		pMotor.rotateToDegrees(135.0, 25.0);

		sleep(500);
		while(!gamepad2.a && !isStopRequested());

		pMotor.rotateToDegrees(180, 25.0);

		sleep(500);
		while(!gamepad2.a && !isStopRequested());

		pMotor.rotateToDegrees(360.0, 25.0);

		sleep(500);
		while(!gamepad2.a && !isStopRequested());

		pMotor.rotateToDegrees(720.0, 25.0);

		sleep(500);
		while(!gamepad2.a && !isStopRequested());
	}
}
