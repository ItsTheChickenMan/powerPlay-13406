package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.TwoWheelTrackingLocalizer;
import org.firstinspires.ftc.teamcode.util.Encoder;

@TeleOp(group = "Tests")
public class RawEncoderVSRoadRunnerTest extends LinearOpMode {
	@Override
	public void runOpMode() throws InterruptedException {
		DcMotorEx parallelEncoder = hardwareMap.get(DcMotorEx.class, "parallelEncoder");
		//Encoder perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "perpendicularEncoder"));

		parallelEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

		SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap){
			// override heading to always be 0 so I don't have to worry about holding the robot perfectly at 0
			@Override
			public double getRawExternalHeading(){
				return 0.0;
			}

			// ditto
			@Override
			public Double getExternalHeadingVelocity() {
				return 0.0;
			}
		};

		while(opModeIsActive()){
			// update localizer
			drive.update();

			// calculate drive position based on encoder position values
			double rawPosition = TwoWheelTrackingLocalizer.encoderTicksToInches((double)parallelEncoder.getCurrentPosition()) * TwoWheelTrackingLocalizer.X_MULTIPLIER;

			// log
			telemetry.addData("rr pose x", drive.getPoseEstimate().getX());
			telemetry.addData("raw pose x", rawPosition);
			telemetry.update();
		}
	}
}
