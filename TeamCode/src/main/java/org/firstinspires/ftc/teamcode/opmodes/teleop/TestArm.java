package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.abe.AbeConstants;
import org.firstinspires.ftc.teamcode.lib.abe.AbeOpMode;

@TeleOp(group = "Tests")
@Config
public class TestArm extends AbeOpMode {
	public static double ELBOW_MOVEMENT = 45.0;
	public static double SLIDES_LENGTH = 36.0;
	public static double WRIST_ANGLE = 35.0;
	public static double WRIST_TEST_ELBOW_MOVEMENT = 60;
	public static double DELAY_MS = 0;

	public static double[] AIM_POSITION_X = new double[]{};

	@Override
	public void runOpMode() throws InterruptedException {
		// initialize abe
		initialize(hardwareMap);

		telemetry.addData("Status", "Initialized");
		telemetry.addLine("These tests do not require or use a controller.  Press play to begin immediately.");
		telemetry.update();

		waitForStart();

		// elbow test...
		logTestMessage("elbow");
		telemetry.addLine("The elbow will rotate up " + ELBOW_MOVEMENT + " degrees, and then down " + ELBOW_MOVEMENT + " degrees...");
		telemetry.update();

		// move up
		this.abe.arm.setElbowAngleDegrees(ELBOW_MOVEMENT);

		this.abe.arm.update(true, false);

		// wait for finish
		while(this.abe.arm.isElbowBusy() && !isStopRequested());
		sleep((long) DELAY_MS);

		// move down
		this.abe.arm.setElbowAngleDegrees(0.0);

		this.abe.arm.update(true, false);

		// wait for finish
		while(this.abe.arm.isElbowBusy() && !isStopRequested());
		sleep((long) DELAY_MS);

		// slides test...
		logTestMessage("slides");
		telemetry.addLine("The slides will extend to " + SLIDES_LENGTH + " inches in length, and then back to base extension (" + AbeConstants.SLIDES_BASE_LENGTH_INCHES + " in.)...");
		telemetry.update();

		// move out
		this.abe.arm.setSlidesLengthInches(SLIDES_LENGTH);

		this.abe.arm.update(false, true);

		// wait for finish
		while(this.abe.arm.areSlidesBusy() && !isStopRequested());
		sleep((long) DELAY_MS);

		// move back
		this.abe.arm.setSlidesLengthInches(AbeConstants.SLIDES_BASE_LENGTH_INCHES);

		this.abe.arm.update(false, true);

		// wait for finish
		while(this.abe.arm.areSlidesBusy() && !isStopRequested());
		sleep((long) DELAY_MS);

		// wrist test...
		logTestMessage("wrist");
		telemetry.addLine("The wrist will stay at " + WRIST_ANGLE + " degrees while the elbow moves up " + WRIST_TEST_ELBOW_MOVEMENT + " degrees and then back " + WRIST_TEST_ELBOW_MOVEMENT + " degrees");
		telemetry.update();

		// set angle
		this.abe.arm.setDesiredWristAngleDegrees(WRIST_ANGLE);

		// move up
		this.abe.arm.setElbowAngleDegrees(WRIST_TEST_ELBOW_MOVEMENT);

		this.abe.arm.update(true, false);

		// wait for finish
		while(this.abe.arm.isElbowBusy() && !isStopRequested());
		sleep((long) DELAY_MS);

		// move back
		this.abe.arm.setElbowAngleDegrees(0.0);

		this.abe.arm.update(true, false);

		// wait for finish
		while(this.abe.arm.isElbowBusy() && !isStopRequested());
		sleep((long) DELAY_MS);

		// aim test...
		logTestMessage("aim + wrist");
		telemetry.addLine("The arm will aim at a set of 9 points while keeping the wrist at " + WRIST_ANGLE + " degrees");
	}

	public void logTestMessage(String test){
		telemetry.addData("Running", test + " test");
	}
}
