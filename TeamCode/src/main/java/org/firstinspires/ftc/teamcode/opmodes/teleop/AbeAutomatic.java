package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.abe.AbeAutonomous;
import org.firstinspires.ftc.teamcode.lib.abe.AbeBot;
import org.firstinspires.ftc.teamcode.lib.abe.AbeConstants;
import org.firstinspires.ftc.teamcode.lib.abe.AbeTeleOp;
import org.firstinspires.ftc.teamcode.lib.utils.GlobalStorage;

@TeleOp(group = "Comp")
public class AbeAutomatic extends AbeTeleOp {
	@Override
	public void runOpMode() throws InterruptedException {
		GlobalStorage.globalTelemetry = telemetry;

		setup();

		// load state
		this.loadStateFromGlobalStorage();

		// note: roadrunner coordinates
		// FIXME: always right
		if(!GlobalStorage.didAuto) {
			setStartPoint(AbeAutonomous.STARTING_POSITION_RIGHT.getX(), AbeAutonomous.STARTING_POSITION_RIGHT.getY(), 0);
		}

		waitForStart();

		while(opModeIsActive()) {
			update();

			telemetry.update();
		}
	}
}