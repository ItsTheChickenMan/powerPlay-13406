package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.abe.AbeOpMode;
import org.firstinspires.ftc.teamcode.lib.utils.GlobalStorage;

@Config
@TeleOp(group = "Tests")
public class AbeSlidesTest extends AbeOpMode {
	@Override
	public void runOpMode() throws InterruptedException {
		GlobalStorage.globalTelemetry = telemetry;

		initialize(hardwareMap);

		//telemetry.addData("Status", "Initialized");
		//telemetry.update();

		for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
			//module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
			telemetry.addData("c info", module.getConnectionInfo());
			telemetry.addData("name", module.getDeviceName());
			telemetry.addLine();
		}

		telemetry.update();

		waitForStart();

		this.abe.clearAim();

		while(!isStopRequested()){
			if(gamepad2.a){
				this.abe.arm.setSlidesLengthInches(SlidesTest.LENGTH_TEST);
			} else {
				this.abe.arm.setSlidesLengthInches(this.abe.arm.getSlidesBaseExtension() + 1.0);
			}

			this.abe.arm.update(false, true);

			telemetry.update();
		}
	}
}
