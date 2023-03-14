package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.lib.abe.AbeAutonomous;

public class AutoCyclingTest extends AbeAutonomous {
	public static Vector2d DEPOSIT_POSITION = new Vector2d(56, 35); // place where the robot should be while depositing

	@Override
	public void runOpMode() throws InterruptedException {
		setup(Mode.RIGHT, new int[]{3, 2}, SampleMecanumDrive.LocalizationType.THREE_WHEEL);

		setPoseEstimate(DEPOSIT_POSITION.getX(), DEPOSIT_POSITION.getY(), 0);

		telemetry.addData("Status", "Initialized");
		telemetry.update();

		waitForStart();

		double startTime = getGlobalTimeSeconds();
		int updates = 0;

		setManualBulkReads();

		while(!gamepad2.b && !isStopRequested()){
			clearBulkCache();

			cycle();

			update();

			/*telemetry.addData("pose estimate", this.abe.drive.getPoseEstimate());
			telemetry.addData("drive steady?", this.abe.drive.isSteady());
			telemetry.addData("elbow steady?", this.abe.arm.isElbowSteady());
			telemetry.addData("slides steady?", this.abe.arm.areSlidesSteady());
			telemetry.addData("elbow error", this.abe.arm.getElbowAngleErrorDegrees());
			telemetry.addData("drive error", this.abe.drive.getAimErrorDegrees());
			telemetry.update();*/

			telemetry.addData("pose estimate", this.abe.drive.getPoseEstimate());
			telemetry.update();

			updates++;
		}

		setAutoBulkReads();

		double duration = getGlobalTimeSeconds() - startTime;

		while(!isStopRequested()){
			telemetry.addData("ms / loop", (duration / updates) * 1000);
			telemetry.addData("loops / second", (updates / duration));
			telemetry.update();
		}
	}
}
