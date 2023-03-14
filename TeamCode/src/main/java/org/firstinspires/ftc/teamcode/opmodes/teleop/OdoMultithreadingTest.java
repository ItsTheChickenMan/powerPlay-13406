package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.lib.abe.AbeOpMode;
import org.firstinspires.ftc.teamcode.lib.utils.TaskThread;

/**
 * doesn't really work...thought it would be fun to try, though
 */
@Config
@TeleOp(group = "Tests")
public class OdoMultithreadingTest extends AbeOpMode {
	public static double LOCALIZATION_TYPE = 1;

	AbeOpMode instance = this;

	TaskThread odometryThread;

	int odoThreadUpdates = 0;

	TaskThread.Actions odometryActions = new TaskThread.Actions() {
		@Override
		public void loop() {
			instance.abe.drive.updateRoadrunnerDrive();

			odoThreadUpdates++;
		}
	};

	@Override
	public void runOpMode() throws InterruptedException {
		initialize(hardwareMap, LOCALIZATION_TYPE > 0 ? SampleMecanumDrive.LocalizationType.THREE_WHEEL : SampleMecanumDrive.LocalizationType.TWO_WHEEL);

		// register odo thread
		this.odometryThread = new TaskThread(odometryActions);

		waitForStart();

		double startTime = getGlobalTimeSeconds();

		this.odometryThread.start();

		int updates = 0;

		while(!gamepad2.a && !isStopRequested()){
			this.abe.drive.driveFieldOriented(-gamepad1.left_stick_y, -gamepad1.left_stick_x, gamepad1.right_stick_x);

			// do something else
			//this.abe.aimAt(24.0, 24.0, 24.0);

			this.abe.updateNoRoadrunner();

			updates++;

			telemetry.addData("pose estimate", this.abe.drive.getPoseEstimate());
			telemetry.update();
		}

		this.odometryThread.stop();

		double duration = getGlobalTimeSeconds();

		while(!isStopRequested()){
			telemetry.addData("main thread ms / loop", (duration / updates) * 1000);
			telemetry.addData("main thread loops / second", updates / duration);

			telemetry.addData("odo thread ms / loop", (duration / odoThreadUpdates) * 1000);
			telemetry.addData("odo thread loops / second", (odoThreadUpdates / duration));

			telemetry.update();
		}
	}
}
