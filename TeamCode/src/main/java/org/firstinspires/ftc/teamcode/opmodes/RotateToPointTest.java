package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.firstinspires.ftc.teamcode.lib.abe.AbeBot;
import org.firstinspires.ftc.teamcode.lib.utils.GlobalStorage;

/**
 * @brief Test opmode for math involving the auto rotate
 */
@TeleOp(group = "Tests")
public class RotateToPointTest extends LinearOpMode {
	private AbeBot abe;

	@Override
	public void runOpMode() throws InterruptedException {
		GlobalStorage.globalTelemetry = telemetry;

		try {
			abe = new AbeBot(hardwareMap);
		} catch( Exception e ){
			telemetry.addLine(e.getMessage());
			telemetry.update();

			waitForStart();

			return;
		}

		this.abe.drive.setModes(DcMotor.RunMode.RUN_USING_ENCODER);

		waitForStart();

		while(opModeIsActive()){
			double forward = -gamepad1.left_stick_y;
			double strafe = gamepad1.left_stick_x;
			double speed = 30.0 * (1.25 - Math.max(gamepad1.left_trigger, gamepad1.right_trigger));

			Vector2d aimVector = new Vector2d(-gamepad1.right_stick_y, -gamepad1.right_stick_x);
			double heading = aimVector.angle();

			// move drive to selected angle, otherwise maintain current angle
			if(aimVector.norm() > 0.5) {
				this.abe.drive.aimAtAngleRadians(heading);
			} else {
				this.abe.drive.aimAtPoint(24, 0);
			}

			this.abe.drive.driveFieldOriented(forward * speed, strafe * speed);
			this.abe.drive.update();

			telemetry.update();
		}
	}
}