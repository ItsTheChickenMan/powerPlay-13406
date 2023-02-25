package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.Encoder;

// nothing fancy, just reports position values
@TeleOp(group = "Tests")
public class EncoderWheelTest extends LinearOpMode {
	@Override
	public void runOpMode() throws InterruptedException {
		// get encoder wheels
		DcMotorEx parallelMotorLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
		DcMotorEx parallelMotorRight = hardwareMap.get(DcMotorEx.class, "backRight");
		DcMotorEx perpendicularMotor = hardwareMap.get(DcMotorEx.class, "frontRight");

		Encoder parallelLeft = new Encoder(parallelMotorLeft);
		Encoder parallelRight = new Encoder(parallelMotorRight);
		Encoder perpendicular = new Encoder(perpendicularMotor);

		telemetry.addData("Status", "Initialized");
		telemetry.update();

		waitForStart();

		while(!isStopRequested()){
			telemetry.addData("parallelLeft", parallelLeft.getCurrentPosition());
			telemetry.addData("parallelLeftVelocity", parallelLeft.getRawVelocity());

			telemetry.addLine();

			telemetry.addData("parallelRight", parallelRight.getCurrentPosition());
			telemetry.addData("parallelRightVelocity", parallelRight.getRawVelocity());

			telemetry.addData("perpendicular", perpendicular.getCurrentPosition());
			telemetry.addData("perpendicularVelocity", perpendicular.getRawVelocity());

			telemetry.update();
		}
	}
}
