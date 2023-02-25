package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.abe.AbeOpMode;
import org.firstinspires.ftc.teamcode.lib.utils.JunctionHelper;

@TeleOp(group = "Tests")
public class GetSagEquations extends AbeOpMode {
	@Override
	public void runOpMode() throws InterruptedException {
		initialize(hardwareMap);

		waitForStart();

		for(JunctionHelper.Level level : JunctionHelper.JUNCTION_LEVELS){
			telemetry.addData(level.toString(), this.abe.arm.getSagCorrectionEquation(level));
		}

		telemetry.update();

		while(!isStopRequested());
	}
}
