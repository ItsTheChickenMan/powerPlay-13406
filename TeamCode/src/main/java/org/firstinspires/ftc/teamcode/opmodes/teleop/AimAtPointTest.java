package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.abe.AbeBot;
import org.firstinspires.ftc.teamcode.lib.abe.AbeConstants;
import org.firstinspires.ftc.teamcode.lib.abe.AbeOpMode;
import org.firstinspires.ftc.teamcode.lib.utils.GlobalStorage;

@TeleOp(group = "Tests")
public class AimAtPointTest extends AbeOpMode {
	@Override
	public void runOpMode() throws InterruptedException {
		GlobalStorage.globalTelemetry = telemetry;

		initializeAbe();

		waitForStart();

		this.abe.aimAt(16, 18, -AbeConstants.ARM_LATERAL_OFFSET_INCHES);

		while(!isStopRequested()){
			double forward = -gamepad1.left_stick_y;
			double strafe = gamepad1.left_stick_x;
			double speed = Math.min(1.0, 1.2 - Math.max(gamepad1.left_trigger, gamepad1.right_trigger))*30;

			this.abe.drive.driveFieldOriented(forward*speed, strafe*speed);

			this.abe.update();

			telemetry.update();
		}
	}
}
