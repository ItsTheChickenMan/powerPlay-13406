package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.lib.abe.AbeTeleOp;
import org.firstinspires.ftc.teamcode.lib.utils.GlobalStorage;

@Config
@TeleOp(group = "Comp")
public class AbeAutomatic extends AbeTeleOp {
	// starting pose used if auto isn't run
	public static Pose2d STARTING_POSE = new Pose2d(9.125, 34.75, 0);

	@Override
	public void runOpMode() throws InterruptedException {
		GlobalStorage.globalTelemetry = telemetry;

		// use two wheel odometry in teleop because positional error is correctable while heading error is not
		setup(SampleMecanumDrive.LocalizationType.TWO_WHEEL);

		// load state from global storage
		this.loadStateFromGlobalStorage();

		// set pose estimate if needed
		// FIXME: needs a left + right side
		if(!GlobalStorage.didAuto){
			this.setPoseEstimate(STARTING_POSE);
		}

		waitForStart();

		while(!isStopRequested()){
			update();

			telemetry.update();
		}
	}
}
