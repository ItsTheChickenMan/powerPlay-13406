package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.lib.abe.AbeOpMode;

@TeleOp(group = "Comp")
public class AbeArmManual extends AbeOpMode {
	@Override
	public void runOpMode() throws InterruptedException {
		initialize(hardwareMap);

		telemetry.addData("Status", "Initialized");
		telemetry.update();

		while(!isStopRequested()){
			boolean clamp = gamepadEx2.a_toggled;

			if(clamp){
				this.abe.arm.setHandClamped();
			} else {
				this.abe.arm.setHandUnclamped();
			}

			double dElbowDegrees = -gamepad2.left_stick_y * (2 - gamepad2.left_trigger) * 10;

			double dSlidesLengthInches = -gamepad2.right_stick_y * (2 - gamepad2.right_trigger) * 12;

			this.abe.arm.setElbowVelocityDegrees(dElbowDegrees);
			this.abe.arm.setSlidesVelocityInches(dSlidesLengthInches);

			this.abe.arm.updateHand();

			updateControllerStates();

			telemetry.addData("elbow angle", this.abe.arm.getElbowAngleDegrees());
			telemetry.addData("slides length", this.abe.arm.getSlidesLengthInches());
			telemetry.update();
		}
	}
}
