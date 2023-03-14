package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.lib.abe.AbeConstants;
import org.firstinspires.ftc.teamcode.lib.utils.GlobalStorage;
import org.firstinspires.ftc.teamcode.lib.utils.PIDControllerRotation;

@TeleOp(group = "Tests")
public class PIDControllerTest extends LinearOpMode {
	@Override
	public void runOpMode() throws InterruptedException {
		GlobalStorage.globalTelemetry = telemetry;

		PIDControllerRotation controller = new PIDControllerRotation(AbeConstants.AIM_HEADING_PID);

		SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap, SampleMecanumDrive.LocalizationType.TWO_WHEEL);

		telemetry.addData("Status", "Initialized");
		telemetry.update();

		waitForStart();

		controller.setTargetPosition(0.0);

		while(!isStopRequested()){
			drive.updatePoseEstimate();

			telemetry.addData("pose heading", drive.getPoseEstimate().getHeading());
			telemetry.addData("error", controller.getLastError());
			telemetry.addData("output", controller.update(drive.getPoseEstimate().getHeading()));
			telemetry.update();
		}
	}
}
