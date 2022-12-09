package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.firstinspires.ftc.teamcode.lib.abe.AbeAutonomous;
import org.firstinspires.ftc.teamcode.lib.utils.GlobalStorage;

// simple test for the cycle from cone stack to a junction (either tall or medium, whichever is statistically more reliable)
@TeleOp(group = "Tests")
public class AutoCyclingTest extends AbeAutonomous {
	@Override
	public void runOpMode() throws InterruptedException {
		GlobalStorage.globalTelemetry = telemetry;

		setup(Mode.RIGHT, new Vector2D(48, 48));

		setStartPoint(34.25, 33.5, Math.toRadians(-90));

		this.abe.arm.unclampFingers();

		waitForStart();

		while(opModeIsActive()) {
			telemetry.addData("drive steady", this.abe.drive.isSteady());
			telemetry.addData("arm steady", this.abe.arm.isSteady());

			if(this.getConesInStack() > 0){
				cycle();
			}

			update();

			telemetry.update();
		}
	}
}
