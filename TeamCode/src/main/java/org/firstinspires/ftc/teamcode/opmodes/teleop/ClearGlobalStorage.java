package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.utils.GlobalStorage;

@TeleOp(name = "Clear Global Storage", group = "AAA")
public class ClearGlobalStorage extends LinearOpMode {

	@Override
	public void runOpMode() throws InterruptedException {

		while(!isStarted()){
			telemetry.addLine("Press Play to clear GlobalStorage");
			telemetry.addLine("Current contents:");
			GlobalStorage.logGlobalStorage(telemetry);

			telemetry.update();
		}

		if(!isStopRequested()){
			GlobalStorage.clearGlobalStorage();

			while(!isStopRequested()){
				telemetry.addLine("Global Storage Cleared");
				telemetry.addLine("Current contents:");

				GlobalStorage.logGlobalStorage(telemetry);

				telemetry.update();
			}
		}
	}
}
