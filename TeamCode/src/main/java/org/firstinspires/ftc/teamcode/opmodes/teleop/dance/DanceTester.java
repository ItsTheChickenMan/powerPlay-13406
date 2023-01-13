package org.firstinspires.ftc.teamcode.opmodes.teleop.dance;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.abe.AbeOpMode;
import org.firstinspires.ftc.teamcode.lib.dance.BeatKeeper;
import org.firstinspires.ftc.teamcode.lib.dance.DanceMoveBitField;
import org.firstinspires.ftc.teamcode.lib.dance.DanceSequence;
import org.firstinspires.ftc.teamcode.lib.utils.PIDControllerRotation;

@TeleOp(group = "zzz")
public class DanceTester extends AbeOpMode {
	@Override
	public void runOpMode() throws InterruptedException {
		/*
		initializeAbe();

		this.abe.drive.pidController = new PIDControllerRotation(50, 0, 0, this.abe.drive.deltaTimer);

		// create dance
		DanceSequence dance = new DanceSequence(98)
						// define moves
						.defineMove("", new DanceMoveBitField()
										.add(DanceMoveBitField.CLAW_BYTES, DanceMoveBitField.CLAW_OPEN)
						)
						;

		waitForStart();

		// wait for user to press a (ensures best synchrony)
		while(!gamepad1.a && opModeIsActive());
		*/
	}
}
