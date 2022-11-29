package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.lib.abe.AbeConstants;
import org.firstinspires.ftc.teamcode.lib.motion.PositionableServo;

@TeleOp(group = "Tests")
public class FingersTest extends LinearOpMode {
	@Override
	public void runOpMode() throws InterruptedException {
		Servo fingerServo = this.hardwareMap.get(Servo.class, "fingerServo");

		//PositionableServo fingers = new PositionableServo(fingerServo, AbeConstants.FINGERS_MAX_RANGE_RADIANS);

		waitForStart();

		//fingerServo.setPosition(1.0);

		while(opModeIsActive()){
			if(gamepad2.a){
				telemetry.addLine("a pressed");
				fingerServo.setPosition(0.8);
			}

			if(gamepad2.y){
				telemetry.addLine("y pressed");
				fingerServo.setPosition(0.2);
			}

			telemetry.addData("servo position", fingerServo.getPosition());
			telemetry.update();

			/*if(gamepad2.a){
				fingers.rotateToDegrees(-fingers.getMaxRangeDegrees()/2.);
			} else {
				fingers.rotateToDegrees(fingers.getMaxRangeDegrees()/2.);
			}*/
			/*if(gamepad2.a) {
				fingerServo.setPosition(0);
			} else {
				fingerServo.setPosition(1);
			}*/
		}
	}
}
