package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.lib.abe.AbeConfig;
import org.firstinspires.ftc.teamcode.lib.abe.AbeConstants;
import org.firstinspires.ftc.teamcode.lib.abe.AbeOpMode;
import org.firstinspires.ftc.teamcode.lib.motion.PositionableMotor;
import org.firstinspires.ftc.teamcode.lib.utils.JunctionHelper;

@TeleOp(group = "Tests")
public class BenchmarkCode extends AbeOpMode {
	@Override
	public void runOpMode() throws InterruptedException {
		initialize(hardwareMap);

		DcMotorEx motor = AbeConfig.loadHardware(hardwareMap).elbowMotor;
		PositionableMotor motorEx = new PositionableMotor(motor, AbeConstants.ELBOW_GEAR_RATIO, AbeConstants.ELBOW_TPR);

		waitForStart();

		double startTime = getGlobalTimeSeconds();
		int updates = 0;

		/*for(LynxModule hub : hardwareMap.getAll(LynxModule.class)){
			hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
		}*/

		while (!gamepad2.a && !isStopRequested()) {
			updates++;
		}

		double duration = getGlobalTimeSeconds() - startTime;

		while (!isStopRequested()) {
			telemetry.addData("ms / loop", (duration / updates) * 1000);
			telemetry.addData("loops / second", (updates / duration));
			telemetry.update();
		}
	}
}