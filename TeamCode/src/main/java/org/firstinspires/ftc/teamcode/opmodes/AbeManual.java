package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.abe.AbeBot;
import org.firstinspires.ftc.teamcode.lib.abe.AbeConstants;
import org.firstinspires.ftc.teamcode.lib.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.lib.motion.AngleAdjuster;

/**
 * Completely manual drive.  Just about every moving part is managed directly
 *
 */

@TeleOp
public class AbeManual extends LinearOpMode {
	// hardware //
	private AbeBot.Hardware hardware;

	// bot //
	private AbeBot abe;

	public void loadHardware(){
		this.hardware = new AbeBot.Hardware();

		this.hardware.frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
		this.hardware.frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
		this.hardware.backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
		this.hardware.backRight = hardwareMap.get(DcMotorEx.class, "backRight");

		//this.hardware.frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
		this.hardware.frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
		//this.hardware.backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
		this.hardware.backRight.setDirection(DcMotorSimple.Direction.REVERSE);

		this.hardware.elbow = hardwareMap.get(DcMotorEx.class, "angleMotor");
		this.hardware.slides = hardwareMap.get(DcMotorEx.class, "slidesMotor");

		this.hardware.slides.setDirection(DcMotorSimple.Direction.REVERSE);

		this.hardware.wristServo = hardwareMap.get(Servo.class, "wristServo");
		this.hardware.fingerServo = hardwareMap.get(Servo.class, "fingerServo");

		this.hardware.elbowLimitSensor = hardwareMap.get(TouchSensor.class, "limitSwitch");

		this.hardware.imu = hardwareMap.get(BNO055IMU.class, "imu");
	}

	@Override
	public void runOpMode() throws InterruptedException {
		// load hardware
		telemetry.addLine("Loading hardware...");
		telemetry.update();

		this.loadHardware();

		// initialize
		try {
			abe = new AbeBot(this.hardware);
		} catch(Exception e){
			telemetry.addLine(e.getMessage());
			telemetry.update();

			return;
		}

		// enable manual
		this.abe.arm.enableManualControl();

		// log
		telemetry.addData("Status", "Initialized");
		telemetry.update();

		// wait for start
		waitForStart();

		// permanent wrist adjustment
		this.abe.arm.positionWristDegrees(25);

		// fingers logic
		boolean lastClamped = false;
		boolean lastDropped = false;

		// delta calculation
		ElapsedTime timer = new ElapsedTime();
		double lastTime = timer.seconds();

		// run op mode
		while(opModeIsActive()) {
			// delta
			double time = timer.seconds();
			double delta = time - lastTime;
			lastTime = time;

			// arm logic //

			// elbow...
			this.abe.arm.setElbowSpeedDegrees(gamepad2.left_stick_y * 50);

			// slides...
			if(Math.abs(gamepad2.right_stick_y) > 0.02) {
				this.abe.arm.unfreezeSlides();
				this.abe.arm.setSlidesSpeed(-gamepad2.right_stick_y * 12);
			} else {
				telemetry.addLine("Freezing");
				this.abe.arm.freezeSlides();
			}

			telemetry.addData("controller", gamepad2.right_stick_y);
			telemetry.addData("speed", this.abe.arm.slides.getVelocity());
			telemetry.addData("extension", this.abe.arm.slides.getExtension());

			// fingers...
			if(gamepad2.a && !lastClamped){
				this.abe.arm.toggleFingers();
			}

			lastClamped = gamepad2.a;

			/*if(gamepad2.x){
				this.abe.arm.positionWristDegrees(25);
			} else {
				this.abe.arm.positionWristDegrees(0);
			}*/

			// wrist...
			telemetry.addData("elbow", this.abe.arm.getElbowAngleDegrees());
			telemetry.update();

			// arm update
			this.abe.arm.update();

			// drive logic //
			double forward = -gamepad1.left_stick_y;
			double strafe = gamepad1.left_stick_x;
			double rotation = gamepad1.right_stick_x;
			double speed = 1.3 - gamepad1.right_trigger;

			this.abe.drive.driveNormal(forward, strafe, rotation, speed);
		}
	}
}