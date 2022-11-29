package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.lib.abe.AbeBot;
import org.firstinspires.ftc.teamcode.lib.utils.AngleHelper;
import org.firstinspires.ftc.teamcode.lib.utils.Vec2;

/**
 * @brief Test opmode for math involving the auto rotate
 */
@TeleOp(group = "Tests")
public class RotateToPointTest extends LinearOpMode {
	private AbeBot abe;

	@Override
	public void runOpMode() throws InterruptedException {
		try {
			abe = new AbeBot(hardwareMap);
		} catch( Exception e ){
			telemetry.addLine(e.getMessage());
			telemetry.update();

			waitForStart();

			return;
		}

		waitForStart();

		this.abe.drive.aimAtPointFromStart(24, 0);

		while(opModeIsActive()){
			double forward = -gamepad1.left_stick_y;
			double strafe = gamepad1.left_stick_x;
			double speed = gamepad1.right_trigger;

			this.abe.drive.driveFieldOriented(forward * speed, strafe * speed);
			this.abe.drive.update();
		}
	}
}