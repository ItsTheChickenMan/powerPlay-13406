package org.firstinspires.ftc.teamcode.opmodes.teleop.dance;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.abe.AbeOpMode;
import org.firstinspires.ftc.teamcode.lib.dance.BeatKeeper;

@TeleOp(group = "zzz")
public class TimingTester extends AbeOpMode {
	@Override
	public void runOpMode() throws InterruptedException {
		/*
		// start up abe
		this.initializeAbe();

		// create beat keeper
		BeatKeeper beatKeeper = new BeatKeeper(150);

		waitForStart();

		beatKeeper.restart();
		beatKeeper.scheduleBeatIn(0);

		while(opModeIsActive()){
			telemetry.addData("totalBeats", beatKeeper.totalBeats());
			telemetry.addData("scheduleBeat", beatKeeper.nextScheduledBeat);
			telemetry.update();

			if(beatKeeper.isFiring()){
				// toggle fingers
				this.abe.arm.toggleFingers();

				// schedule next
				beatKeeper.scheduleBeatIn(2);
			}
		}
		*/
	}
}