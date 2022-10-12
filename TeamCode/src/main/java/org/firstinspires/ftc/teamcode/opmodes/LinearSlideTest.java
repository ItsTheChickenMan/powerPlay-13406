package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.lib.motion.LinearSlides;
import org.firstinspires.ftc.teamcode.lib.motion.PositionableMotor;

@TeleOp
public class LinearSlideTest extends LinearOpMode {
	public void runOpMode(){
		DcMotorEx drive = hardwareMap.get(DcMotorEx.class, "slidesMotor");

		drive.setDirection(DcMotorSimple.Direction.REVERSE);

		LinearSlides slides = new LinearSlides(new PositionableMotor(drive, 1, 384.5), 38.4, 976.0/112.0);

		telemetry.addData("Status", "Initialized");
		telemetry.update();

		waitForStart();

		slides.extendTo(16, 32);

		while(opModeIsActive()){
			telemetry.addData("Status", "Running");
			telemetry.update();
		}
	}
}
