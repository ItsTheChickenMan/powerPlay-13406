package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.lib.abe.AbeOpMode;
import org.firstinspires.ftc.teamcode.lib.utils.GlobalStorage;

@Config
@TeleOp(group = "Tests")
public class AimAtPointTest extends AbeOpMode {
	public static double AIM_POINT_X = 24;
	public static double AIM_POINT_Y = 0;
	public static double USE_THETA_FF = -1.0; // use thetaFF if >0
	public static double DELAY = 50;

	@Override
	public void runOpMode() throws InterruptedException {
		GlobalStorage.globalTelemetry = telemetry;

		initialize(hardwareMap, SampleMecanumDrive.LocalizationType.TWO_WHEEL);

		telemetry.addData("Status", "Initialized");
		telemetry.update();

		waitForStart();

		while(!isStopRequested()){
			this.abe.drive.driveFieldOriented(
							-gamepad1.left_stick_y,
							-gamepad1.left_stick_x,
							gamepad1.right_stick_x
			);

			this.abe.drive.aimAtPoint(AIM_POINT_X, AIM_POINT_Y);

			this.abe.drive.update(USE_THETA_FF >= 0.0);

			telemetry.addData("point", AIM_POINT_X + ", " + AIM_POINT_Y);
			telemetry.addData("error (degrees)", Math.toDegrees(this.abe.drive.getAimErrorRadians()));
			telemetry.update();

			sleep((long)DELAY);
		}
	}
}
