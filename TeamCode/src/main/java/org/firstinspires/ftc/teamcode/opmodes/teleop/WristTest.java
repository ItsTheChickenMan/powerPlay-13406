package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.motion.AngleAdjuster;
import org.firstinspires.ftc.teamcode.lib.motion.PositionableMotor;
import org.firstinspires.ftc.teamcode.lib.motion.PositionableServo;

@TeleOp(group = "Tests")
public class WristTest extends LinearOpMode {
	PositionableServo wrist;
	PositionableMotor angleAdjuster;

	TouchSensor limitSensor;

	public void loadHardware(){
		Servo servo = hardwareMap.get(Servo.class, "wristServo");

		//servo.setDirection(Servo.Direction.REVERSE);

		this.wrist = new PositionableServo(servo, Math.toRadians(360));
		this.limitSensor = hardwareMap.get(TouchSensor.class, "limitSwitch");
		this.angleAdjuster = new AngleAdjuster(hardwareMap.get(DcMotorEx.class, "angleMotor"), 24.0, 384.5, this.limitSensor);
	}

	@Override
	public void runOpMode() throws InterruptedException {
		this.loadHardware();

		waitForStart();

		ElapsedTime timer = new ElapsedTime();

		double delta = 0;
		double lastTime = timer.seconds();

		while(opModeIsActive()){
			double time = timer.seconds();
			delta = time - lastTime;
			lastTime = time;

			this.angleAdjuster.rotateSpeedDegrees(gamepad1.left_stick_y * 30);

			telemetry.addData("angle", this.angleAdjuster.getAngleDegrees());
			telemetry.update();

			this.wrist.rotateToDegrees(-wrist.getMaxRangeDegrees()/2. - this.angleAdjuster.getAngleDegrees());
		}
	}
}
