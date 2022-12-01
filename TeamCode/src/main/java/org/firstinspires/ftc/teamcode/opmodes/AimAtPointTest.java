package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.lib.abe.AbeBot;
import org.firstinspires.ftc.teamcode.lib.abe.AbeConfig;
import org.firstinspires.ftc.teamcode.lib.abe.AbeConstants;
import org.firstinspires.ftc.teamcode.lib.abe.AbeDrive;

@TeleOp(group = "Tests")
public class AimAtPointTest extends LinearOpMode {
	//private AbeDrive drive;
	private AbeBot abe;

	public static Telemetry globalTelemetry;

	@Override
	public void runOpMode() throws InterruptedException {
		// cheap hack to get telemetry global
		AimAtPointTest.globalTelemetry = telemetry;

		// load drive train
		//this.abe = new AbeDrive(hardwareMap, AbeConstants.ARM_OFFSET, 0.5, 0.0, 0.025);
		try {
			this.abe = new AbeBot(hardwareMap);
		} catch (Exception e) {
			telemetry.addLine("hardware is missing");
			telemetry.update();

			waitForStart();

			return;
		}

		// set point to 0
		this.abe.aimAtPointFromStart(0, 28.5, 12);

		waitForStart();

		/*ElapsedTime timer = new ElapsedTime();

		double cycle = 0;

		this.drive.setPointFromStart(100, 0);

		while(!isStopRequested()){
			telemetry.addData("timer", timer.seconds());

			if(timer.seconds() > 6.0) {
				this.drive.setPointFromStart(Math.cos(Math.PI * cycle) * 100, Math.sin(Math.PI * cycle) * 100);

				cycle += 0.5;

				timer.reset();
			}

			this.drive.update(telemetry);
		}*/

		while(!isStopRequested()){
			// get desired movement
			double forward = -gamepad1.left_stick_y;
			double strafe = gamepad1.left_stick_x;
			double slow = 1.3 - gamepad1.right_trigger;

			// apply slowmode
			forward *= slow;
			strafe *= slow;

			// apply field oriented drive
			this.abe.drive.driveFieldOriented(forward, strafe);

			// update abe
			this.abe.update();

			telemetry.update();
		}
	}
}
