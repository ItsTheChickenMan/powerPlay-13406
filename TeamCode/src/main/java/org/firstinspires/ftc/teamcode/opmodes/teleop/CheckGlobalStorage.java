package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.utils.GlobalStorage;

@TeleOp(name = "Check Global Storage", group = "AAA")
public class CheckGlobalStorage extends LinearOpMode {
	@Override
	public void runOpMode() throws InterruptedException {
		while(!isStarted() && !isStopRequested()){
			telemetry.addData("globalTelemetry?", GlobalStorage.globalTelemetry != null);
			telemetry.addData("current pose", GlobalStorage.currentPose != null ? GlobalStorage.currentPose.toString() : "null");
			telemetry.addData("current elbow angle", GlobalStorage.currentElbowAngleRadians);
			telemetry.addData("current slides extension", GlobalStorage.currentSlidesExtension);

			telemetry.update();
		}
	}
}
