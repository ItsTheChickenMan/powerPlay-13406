package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.utils.GlobalStorage;

@TeleOp(name = "Check Global Storage", group = "AAA")
public class CheckGlobalStorage extends LinearOpMode {
	@Override
	public void runOpMode() throws InterruptedException {
		while(!isStarted() && !isStopRequested()){
			GlobalStorage.logGlobalStorage(telemetry);

			telemetry.update();
		}
	}
}
