package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@TeleOp(group = "drive")
public class LaggyLocalizationTest extends LinearOpMode {
	public static double LAG_AMOUNT = 50; // milliseconds

	@Override
	public void runOpMode() throws InterruptedException {
		telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

		SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

		drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

		waitForStart();

		ElapsedTime timer = new ElapsedTime();

		double counter = LAG_AMOUNT;

		while (!isStopRequested()) {
			drive.setWeightedDrivePower(
							new Pose2d(
											-gamepad1.left_stick_y*0.5,
											-gamepad1.left_stick_x*0.5,
											-gamepad1.right_stick_x*0.5
							)
			);

			drive.update();

			/*drive.updateNoOdo();

			if(timer.milliseconds() >= counter){
				drive.updatePoseEstimate();

				counter += LAG_AMOUNT;
			}*/

			Pose2d poseEstimate = drive.getCorrectedPoseEstimate();
			telemetry.addData("x", poseEstimate.getX());
			telemetry.addData("y", poseEstimate.getY());
			telemetry.addData("heading", poseEstimate.getHeading());
			telemetry.update();

			sleep((long)LAG_AMOUNT);
		}
	}
}
