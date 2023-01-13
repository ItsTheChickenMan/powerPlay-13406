package org.firstinspires.ftc.teamcode.opmodes.teleop.dance;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.abe.AbeOpMode;
import org.firstinspires.ftc.teamcode.lib.dance.DanceMoveBitField;
import org.firstinspires.ftc.teamcode.lib.dance.DanceSequence;
import org.firstinspires.ftc.teamcode.lib.utils.PIDControllerRotation;

@TeleOp(group = "zzz")
public class SpaceCowboy extends AbeOpMode {
	@Override
	public void runOpMode() throws InterruptedException {
		/*
		initializeAbe();

		this.abe.drive.pidController = new PIDControllerRotation(50, 0, 0, this.abe.drive.deltaTimer);

		// create dance
		DanceSequence spaceCowbowDance = new DanceSequence(99*4) // *4 to be able to do sixteenth note timing
						// define moves
						.defineMove("default", new DanceMoveBitField())
						.defineMove("OP1_clawOpenSlidesIn", new DanceMoveBitField()
										.add(DanceMoveBitField.CLAW_BYTES, DanceMoveBitField.CLAW_OPEN)
										.add(DanceMoveBitField.SLIDES_BYTES, DanceMoveBitField.SLIDES_000)
										.add(DanceMoveBitField.WRIST_BYTES, DanceMoveBitField.WRIST_000)
						)
						.defineMove("OP1_clawClosedSlidesIn", new DanceMoveBitField()
										.add(DanceMoveBitField.CLAW_BYTES, DanceMoveBitField.CLAW_CLOSED)
										.add(DanceMoveBitField.SLIDES_BYTES, DanceMoveBitField.SLIDES_000)
										.add(DanceMoveBitField.WRIST_BYTES, DanceMoveBitField.WRIST_000)
						)
						.defineMove("OP1_clawClosedSlidesOutALittle", new DanceMoveBitField()
										.add(DanceMoveBitField.CLAW_BYTES, DanceMoveBitField.CLAW_CLOSED)
										.add(DanceMoveBitField.SLIDES_BYTES, DanceMoveBitField.SLIDES_015)
										.add(DanceMoveBitField.WRIST_BYTES, DanceMoveBitField.WRIST_000)
						)
						.defineMove("OP1_clawOpenSlidesOutALittle", new DanceMoveBitField()
										.add(DanceMoveBitField.CLAW_BYTES, DanceMoveBitField.CLAW_OPEN)
										.add(DanceMoveBitField.SLIDES_BYTES, DanceMoveBitField.SLIDES_015)
										.add(DanceMoveBitField.WRIST_BYTES, DanceMoveBitField.WRIST_000)
						)
						.defineMove("OP1_clawOpenWristDownSlidesIn", new DanceMoveBitField()
										.add(DanceMoveBitField.CLAW_BYTES, DanceMoveBitField.CLAW_OPEN)
										.add(DanceMoveBitField.SLIDES_BYTES, DanceMoveBitField.SLIDES_000)
										.add(DanceMoveBitField.WRIST_BYTES, DanceMoveBitField.WRIST_060)
						)

						.defineMove("VE1_slideForwardLeft", new DanceMoveBitField()
										.add(DanceMoveBitField.CLAW_BYTES, DanceMoveBitField.CLAW_OPEN)
										.add(DanceMoveBitField.DRIVE_ROTATION_BYTES, DanceMoveBitField.DRIVE_ROTATION_045)
										.add(DanceMoveBitField.DRIVE_TRANSLATION_BYTES, DanceMoveBitField.DRIVE_TRANSLATION_FORWARD)
						)
						.defineMove("VE1_slideForwardRight", new DanceMoveBitField()
										.add(DanceMoveBitField.CLAW_BYTES, DanceMoveBitField.CLAW_CLOSED)
										.add(DanceMoveBitField.DRIVE_ROTATION_BYTES, DanceMoveBitField.DRIVE_ROTATION_315)
										.add(DanceMoveBitField.DRIVE_TRANSLATION_BYTES, DanceMoveBitField.DRIVE_TRANSLATION_FORWARD)
						)
						.defineMove("VE1_slideBackLeft", new DanceMoveBitField()
										.add(DanceMoveBitField.CLAW_BYTES, DanceMoveBitField.CLAW_OPEN)
										.add(DanceMoveBitField.DRIVE_ROTATION_BYTES, DanceMoveBitField.DRIVE_ROTATION_045)
										.add(DanceMoveBitField.DRIVE_TRANSLATION_BYTES, DanceMoveBitField.DRIVE_TRANSLATION_BACKWARD)
						)
						.defineMove("VE1_slideBackRight", new DanceMoveBitField()
										.add(DanceMoveBitField.CLAW_BYTES, DanceMoveBitField.CLAW_CLOSED)
										.add(DanceMoveBitField.DRIVE_ROTATION_BYTES, DanceMoveBitField.DRIVE_ROTATION_315)
										.add(DanceMoveBitField.DRIVE_TRANSLATION_BYTES, DanceMoveBitField.DRIVE_TRANSLATION_BACKWARD)
						)
						.defineMove("VE1_slideLeftSideLeft", new DanceMoveBitField()
										.add(DanceMoveBitField.CLAW_BYTES, DanceMoveBitField.CLAW_OPEN)
										.add(DanceMoveBitField.DRIVE_ROTATION_BYTES, DanceMoveBitField.DRIVE_ROTATION_045)
										.add(DanceMoveBitField.DRIVE_TRANSLATION_BYTES, DanceMoveBitField.DRIVE_TRANSLATION_LEFT)
						)
						.defineMove("VE1_slideLeftSideRight", new DanceMoveBitField()
										.add(DanceMoveBitField.CLAW_BYTES, DanceMoveBitField.CLAW_CLOSED)
										.add(DanceMoveBitField.DRIVE_ROTATION_BYTES, DanceMoveBitField.DRIVE_ROTATION_315)
										.add(DanceMoveBitField.DRIVE_TRANSLATION_BYTES, DanceMoveBitField.DRIVE_TRANSLATION_LEFT)
						)
						.defineMove("VE1_slideRightSideLeft", new DanceMoveBitField()
										.add(DanceMoveBitField.CLAW_BYTES, DanceMoveBitField.CLAW_OPEN)
										.add(DanceMoveBitField.DRIVE_ROTATION_BYTES, DanceMoveBitField.DRIVE_ROTATION_045)
										.add(DanceMoveBitField.DRIVE_TRANSLATION_BYTES, DanceMoveBitField.DRIVE_TRANSLATION_RIGHT)
						)
						.defineMove("VE1_slideRightSideRight", new DanceMoveBitField()
										.add(DanceMoveBitField.CLAW_BYTES, DanceMoveBitField.CLAW_CLOSED)
										.add(DanceMoveBitField.DRIVE_ROTATION_BYTES, DanceMoveBitField.DRIVE_ROTATION_315)
										.add(DanceMoveBitField.DRIVE_TRANSLATION_BYTES, DanceMoveBitField.DRIVE_TRANSLATION_RIGHT)
						)
						.defineMove("VE1_lookLeft", new DanceMoveBitField()
										.add(DanceMoveBitField.CLAW_BYTES, DanceMoveBitField.CLAW_OPEN)
										.add(DanceMoveBitField.DRIVE_ROTATION_BYTES, DanceMoveBitField.DRIVE_ROTATION_045)
						)
						.defineMove("VE1_lookRight", new DanceMoveBitField()
										.add(DanceMoveBitField.CLAW_BYTES, DanceMoveBitField.CLAW_CLOSED)
										.add(DanceMoveBitField.DRIVE_ROTATION_BYTES, DanceMoveBitField.DRIVE_ROTATION_315)
						)
						.defineMove("VE1_lookLeftSlidesOut", new DanceMoveBitField()
										.add(DanceMoveBitField.CLAW_BYTES, DanceMoveBitField.CLAW_CLOSED)
										.add(DanceMoveBitField.DRIVE_ROTATION_BYTES, DanceMoveBitField.DRIVE_ROTATION_045)
										.add(DanceMoveBitField.SLIDES_BYTES, DanceMoveBitField.SLIDES_015)
						)
						.defineMove("VE1_lookRightSlidesOut", new DanceMoveBitField()
										.add(DanceMoveBitField.CLAW_BYTES, DanceMoveBitField.CLAW_CLOSED)
										.add(DanceMoveBitField.DRIVE_ROTATION_BYTES, DanceMoveBitField.DRIVE_ROTATION_315)
										.add(DanceMoveBitField.SLIDES_BYTES, DanceMoveBitField.SLIDES_015)
						)

						.defineMove("RE1_moveBackSlidesOutWristDown", new DanceMoveBitField()
										.add(DanceMoveBitField.WRIST_BYTES, DanceMoveBitField.WRIST_060)
										.add(DanceMoveBitField.DRIVE_TRANSLATION_BYTES, DanceMoveBitField.DRIVE_TRANSLATION_BACKWARD)
										.add(DanceMoveBitField.SLIDES_BYTES, DanceMoveBitField.SLIDES_045)
						)
						.defineMove("RE1_moveBackSlidesOutWristUp", new DanceMoveBitField()
										.add(DanceMoveBitField.WRIST_BYTES, DanceMoveBitField.WRIST_000)
										.add(DanceMoveBitField.DRIVE_TRANSLATION_BYTES, DanceMoveBitField.DRIVE_TRANSLATION_BACKWARD)
										.add(DanceMoveBitField.SLIDES_BYTES, DanceMoveBitField.SLIDES_045)
						)
						.defineMove("RE1_moveBackSlidesInWristDown", new DanceMoveBitField()
										.add(DanceMoveBitField.WRIST_BYTES, DanceMoveBitField.WRIST_060)
										.add(DanceMoveBitField.DRIVE_TRANSLATION_BYTES, DanceMoveBitField.DRIVE_TRANSLATION_BACKWARD)
										.add(DanceMoveBitField.SLIDES_BYTES, DanceMoveBitField.SLIDES_000)
						)
						.defineMove("RE1_moveBackSlidesInWristUp", new DanceMoveBitField()
										.add(DanceMoveBitField.WRIST_BYTES, DanceMoveBitField.WRIST_000)
										.add(DanceMoveBitField.DRIVE_TRANSLATION_BYTES, DanceMoveBitField.DRIVE_TRANSLATION_BACKWARD)
										.add(DanceMoveBitField.SLIDES_BYTES, DanceMoveBitField.SLIDES_000)
						)
						.defineMove("RE1_moveForwardSlidesOutWristDown", new DanceMoveBitField()
										.add(DanceMoveBitField.WRIST_BYTES, DanceMoveBitField.WRIST_060)
										.add(DanceMoveBitField.DRIVE_TRANSLATION_BYTES, DanceMoveBitField.DRIVE_TRANSLATION_FORWARD)
										.add(DanceMoveBitField.SLIDES_BYTES, DanceMoveBitField.SLIDES_045)
						)
						.defineMove("RE1_moveForwardSlidesOutWristUp", new DanceMoveBitField()
										.add(DanceMoveBitField.WRIST_BYTES, DanceMoveBitField.WRIST_000)
										.add(DanceMoveBitField.DRIVE_TRANSLATION_BYTES, DanceMoveBitField.DRIVE_TRANSLATION_FORWARD)
										.add(DanceMoveBitField.SLIDES_BYTES, DanceMoveBitField.SLIDES_045)
						)
						.defineMove("RE1_moveForwardSlidesInWristDown", new DanceMoveBitField()
										.add(DanceMoveBitField.WRIST_BYTES, DanceMoveBitField.WRIST_060)
										.add(DanceMoveBitField.DRIVE_TRANSLATION_BYTES, DanceMoveBitField.DRIVE_TRANSLATION_FORWARD)
										.add(DanceMoveBitField.SLIDES_BYTES, DanceMoveBitField.SLIDES_000)
						)
						.defineMove("RE1_moveForwardSlidesInWristUp", new DanceMoveBitField()
										.add(DanceMoveBitField.WRIST_BYTES, DanceMoveBitField.WRIST_000)
										.add(DanceMoveBitField.DRIVE_TRANSLATION_BYTES, DanceMoveBitField.DRIVE_TRANSLATION_FORWARD)
										.add(DanceMoveBitField.SLIDES_BYTES, DanceMoveBitField.SLIDES_000)
						)

						// define dance

						// (first eight beats are simple open/close action, don't want to get too crazy too fast)
						.addMoveToSequence("OP1_clawOpenSlidesIn", 4)
						.addMoveToSequence("OP1_clawClosedSlidesIn", 4)

						.loopLastMoves(2)
						.loopLastMoves(2)
						.loopLastMoves(2)

						// (next four beats are simple slide action)
						.addMoveToSequence("OP1_clawClosedSlidesOutALittle", 4)
						.addMoveToSequence("OP1_clawClosedSlidesIn", 4)

						.loopLastMoves(2)

						// (do a little claw trick to match the timing of the cowboy sting)
						.addMoveToSequence("OP1_clawOpenSlidesIn", 1)
						.addMoveToSequence("OP1_clawClosedSlidesIn", 1)
						.addMoveToSequence("OP1_clawOpenSlidesIn", 1)
						.addMoveToSequence("OP1_clawClosedSlidesIn", 1)
						.addMoveToSequence("OP1_clawOpenSlidesIn", 4)

						// (continue slide action)
						.addMoveToSequence("OP1_clawClosedSlidesOutALittle", 4)
						.addMoveToSequence("OP1_clawClosedSlidesIn", 4)

						.loopLastMoves(19)

						// (get a little more funky, combo slides + claw action)
						.addMoveToSequence("OP1_clawOpenSlidesOutALittle", 4)
						.addMoveToSequence("OP1_clawClosedSlidesIn", 4)

						.loopLastMoves(2)
						.loopLastMoves(2)
						.loopLastMoves(2)

						// (reverse)
						.addMoveToSequence("OP1_clawClosedSlidesOutALittle", 4)
						.addMoveToSequence("OP1_clawOpenSlidesIn", 4)

						.loopLastMoves(2)

						// (mess with wrist for cowboy sting)
						.addMoveToSequence("OP1_clawOpenWristDownSlidesIn", 1)
						.addMoveToSequence("OP1_clawOpenSlidesIn", 1)
						.addMoveToSequence("OP1_clawOpenWristDownSlidesIn", 1)
						.addMoveToSequence("OP1_clawOpenSlidesIn", 1)
						.addMoveToSequence("OP1_clawOpenWristDownSlidesIn", 4)

						// (continue reverse)
						.addMoveToSequence("OP1_clawClosedSlidesOutALittle", 4)
						.addMoveToSequence("OP1_clawOpenSlidesIn", 4)

						.loopLastMoves(19)

						// (once upon a time...)
						.addMoveToSequence("VE1_slideBackLeft", 8)
						.addMoveToSequence("VE1_slideBackRight", 8)

						// (there used to be...)
						.addMoveToSequence("VE1_slideForwardLeft", 8)
						.addMoveToSequence("VE1_slideForwardRight", 8)

						// (a space cowboy...from this)
						.addMoveToSequence("VE1_slideLeftSideLeft", 8)
						.addMoveToSequence("VE1_slideLeftSideRight", 8)

						// (galaxy...he had)
						.addMoveToSequence("VE1_lookLeft", 8)
						.addMoveToSequence("OP1_clawClosedSlidesIn", 8)

						// (two laser guns...with)
						//.addMoveToSequence("OP1_clawClosedSlidesOutALittle", 2) // two
						//.addMoveToSequence("OP1_clawClosedSlidesIn", 2) // la-
						.addMoveToSequence("OP1_clawOpenWristDownSlidesIn", 2) // two
						.addMoveToSequence("OP1_clawOpenSlidesIn", 2)

						.loopLastMoves(2) // -ser guns

						.addMoveToSequence("OP1_clawClosedSlidesIn", 8)

						// (heat like the sun?...)
						.addMoveToSequence("OP1_clawOpenSlidesIn", 2) // beat
						.addMoveToSequence("OP1_clawClosedSlidesIn", 2) // like

						.loopLastMoves(2) // the sun...

						.addMoveToSequence("OP1_clawClosedSlidesIn", 8)

						// (something the guys...are)
						.addMoveToSequence("VE1_lookLeft", 4)
						.addMoveToSequence("VE1_lookLeftSlidesOut", 4)
						.addMoveToSequence("VE1_lookRightSlidesOut", 4)
						.addMoveToSequence("VE1_lookRight", 4)

						// (here? just for fun...[space cowboy])
						.loopLastMoves(4)

						// (he's bad...he's mean...he's the)
						.addMoveToSequence("VE1_lookLeftSlidesOut", 8)
						.addMoveToSequence("VE1_lookRightSlidesOut", 8)

						// (space cowboy with the spacey jeans, he's)
						.addMoveToSequence("RE1_moveBackSlidesOutWristDown", 4)
						.addMoveToSequence("RE1_moveBackSlidesInWristUp", 4)
						.addMoveToSequence("RE1_moveForwardSlidesOutWristDown", 4)
						.addMoveToSequence("RE1_moveForwardSlidesInWristUp", 4)

						// (bad...he's number one, he's the)
						.addMoveToSequence("VE1_slideRightSideLeft", 8)
						.addMoveToSequence("VE1_slideRightSideRight", 8)

						// (space cowboy with the laser guns, he's)
						.addMoveToSequence("RE1_moveForwardSlidesOutWristDown", 4)
						.addMoveToSequence("RE1_moveForwardSlidesInWristUp", 4)
						.addMoveToSequence("RE1_moveBackSlidesOutWristDown", 4)
						.addMoveToSequence("RE1_moveBackSlidesInWristUp", 4)

						// (bad, he's mean, he's the space cowboy with the spacey jeans, he's)
						.repeat(12, 7)

						// (bad, he's number one, he's the)
						.addMoveToSequence("VE1_lookLeft", 4)
						.addMoveToSequence("VE1_lookLeftSlidesOut", 4)
						.addMoveToSequence("VE1_lookRightSlidesOut", 4)
						.addMoveToSequence("VE1_lookRight", 4)

						// (space cowboy with the laser guns)
						.repeat(14, 11)

						// (repeat everything for verse 2 + refrain 2)
						.loopLastMoves(52)

						// (bridge time)
						.addMoveToSequence("OP1_clawClosedSlidesIn", 16 + 2) // wait a bit extra  to account for weird skip earlier

						// (yippee yai yippee yai yippee yai yay)
						.addMoveToSequence("VE1_lookLeft", 4)
						.addMoveToSequence("VE1_lookRight", 4)

						.loopLastMoves(2)

						// (pause...)
						.addMoveToSequence("OP1_clawClosedSlidesIn", 16)

						// (howdy hey hey)
						.addMoveToSequence("RE1_moveForwardSlidesOutWristUp", 4)
						.addMoveToSequence("RE1_moveForwardSlidesOutWristDown", 4)

						.loopLastMoves(2)

						// (yippee yai again)
						.repeat(10, 5)

						// (howdy hey hey)
						.addMoveToSequence("RE1_moveBackSlidesOutWristUp", 4)
						.addMoveToSequence("RE1_moveBackSlidesOutWristDown", 4)

						.loopLastMoves(2)

						// (simplified beat)
						//.repeatFromStart(0, 18)

						//.loopLastMoves(19)

						// (repeat verse for weird bridge 2)
						// note: 76

						// end
						.addMoveToSequence("default", 9999)
						;

		waitForStart();

		// wait for user to press a (ensures best synchrony)
		while(!gamepad1.a && opModeIsActive());

		spaceCowbowDance.dance(this.abe, this, false);

		while(opModeIsActive());
		*/
	}
}
