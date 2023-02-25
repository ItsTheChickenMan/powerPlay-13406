package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.lib.abe.AbeDrive;
import org.firstinspires.ftc.teamcode.lib.abe.AbeOpMode;
import org.firstinspires.ftc.teamcode.lib.utils.AngleHelper;
import org.firstinspires.ftc.teamcode.lib.utils.GlobalStorage;

@TeleOp(group = "Tests")
@Config
public class RotationPIDTest extends AbeOpMode {
	public static double ROTATION_AMOUNT = 90.0;
	public static double ROTATION_INTERVAL = 3.5;

	@Override
	public void runOpMode() throws InterruptedException {
		GlobalStorage.globalTelemetry = telemetry;

		initialize(hardwareMap);

		// create drive
		AbeDrive drive = new AbeDrive(hardwareMap, SampleMecanumDrive.LocalizationType.THREE_WHEEL);

		telemetry.addData("Status", "Initialized");
		telemetry.update();

		waitForStart();

		double currentAngle = 0;

		double switchSchedule = getScheduledTime(ROTATION_INTERVAL);

		// start rotating
		while(!isStopRequested()){
			// check aim switch
			if(isEventFiring(switchSchedule)){
				currentAngle += Math.toRadians(ROTATION_AMOUNT);

				switchSchedule = getScheduledTime(ROTATION_INTERVAL);
			}

			drive.aimAtAngleRadians(currentAngle);

			drive.update();

			Pose2d poseEstimate = drive.getPoseEstimate();

			double heading = poseEstimate.getHeading();

			double errorRadians = Math.abs(AngleHelper.angularDistanceRadians(heading, currentAngle));

			telemetry.addData("currentAngle", currentAngle);
			telemetry.addData("current heading", heading);
			telemetry.addData("error (degrees)", Math.toDegrees(errorRadians));
			telemetry.update();
		}
	}
}
