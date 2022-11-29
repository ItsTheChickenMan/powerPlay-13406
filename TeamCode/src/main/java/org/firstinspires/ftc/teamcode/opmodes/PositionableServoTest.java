package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.lib.motion.PositionableServo;

@TeleOp(group = "Tests")
public class PositionableServoTest extends LinearOpMode {
	public PositionableServo servo;

	public void loadHardware(){
		servo = new PositionableServo(hardwareMap.get(Servo.class, "wristServo"));
	}

	@Override
	public void runOpMode() throws InterruptedException {
		// init
		this.loadHardware();

		waitForStart();

		this.servo.rotateToDegrees(-150);

		while(opModeIsActive()){

		}
	}
}
