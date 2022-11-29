package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.abe.AbeBot;
import org.firstinspires.ftc.teamcode.lib.abe.AbeConstants;

/**
 * @brief Test how stressed the arm is
 */
@TeleOp(group = "Tests")
public class ArmStressTest extends LinearOpMode {
	private AbeBot abe;

	@Override
	public void runOpMode() throws InterruptedException {
		try {
			abe = new AbeBot(hardwareMap);
		} catch(Exception e) {
			telemetry.addLine("hardware is missing");
			telemetry.update();

			waitForStart();

			return;
		}

		waitForStart();

		double heights[] = {9.15, 16.0, 24.0, 32.0, 37.0};
		double offsets[] = {18.0, 24.0, 30.0};
		//double offsets[] = {5.0, 8.0, 11.0, 14.0, 17.0, 20.0};
		int currentOffset = 0;
		int currentHeightOffset = 0;

		boolean lastPressed = false;
		boolean lastPressedB = false;

		while(!gamepad2.right_bumper && !isStopRequested()){
			if(gamepad2.a && !lastPressed){
				currentOffset++;

				currentOffset %= (offsets.length);
			}

			if(gamepad2.b && !lastPressedB){
				currentHeightOffset++;
				currentHeightOffset %= heights.length;
			}

			lastPressed = gamepad2.a;
			lastPressedB = gamepad2.b;

			this.abe.arm.aimAt(offsets[currentOffset], heights[currentHeightOffset] - AbeConstants.ARM_VERTICAL_OFFSET_INCHES);

			this.abe.arm.update();

			telemetry.addData("angle", this.abe.arm.getElbowAngleDegrees());
			telemetry.update();
		}

		this.abe.arm.setManualControl(true);

		this.abe.arm.setElbowAngleDegrees(0.0, 35.0);

		while(!gamepad2.left_bumper && !isStopRequested()){
			telemetry.addData("angle", this.abe.arm.getElbowAngleDegrees());
			telemetry.update();
		};
	}
}