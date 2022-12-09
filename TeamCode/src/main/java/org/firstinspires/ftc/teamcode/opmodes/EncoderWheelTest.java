package org.firstinspires.ftc.teamcode.opmodes;

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
		DcMotorEx parallelMotor = hardwareMap.get(DcMotorEx.class, "parallelEncoder");
		DcMotorEx perpendicularMotor = hardwareMap.get(DcMotorEx.class, "perpendicularEncoder");

		Encoder parallel = new Encoder(parallelMotor);
		Encoder perpendicular = new Encoder(perpendicularMotor);

		telemetry.addData("Status", "Initialized");
		telemetry.update();

		waitForStart();

		while(!isStopRequested()){
			telemetry.addData("parallel", parallel.getCurrentPosition());
			telemetry.addData("parallelVelocity", parallel.getCorrectedVelocity());

			telemetry.addData("perpendicular", perpendicular.getCurrentPosition());
			telemetry.addData("perpendicularVelocity", perpendicular.getCorrectedVelocity());

			telemetry.update();
		}
	}
}
