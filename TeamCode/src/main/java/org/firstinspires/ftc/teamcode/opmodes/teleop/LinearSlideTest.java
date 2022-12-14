package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.lib.abe.AbeConstants;
import org.firstinspires.ftc.teamcode.lib.motion.LinearSlides;
import org.firstinspires.ftc.teamcode.lib.motion.PositionableMotor;

@TeleOp(group = "Tests")
public class LinearSlideTest extends LinearOpMode {
	public void runOpMode(){
		DcMotorEx drive = hardwareMap.get(DcMotorEx.class, "slidesMotor");

		drive.setDirection(DcMotorSimple.Direction.REVERSE);

		LinearSlides slides = new LinearSlides(new PositionableMotor(drive, 1, 384.5), AbeConstants.SLIDE_BASE_LENGTH_INCHES, AbeConstants.SLIDE_BASE_LENGTH_INCHES + AbeConstants.SLIDE_MAX_EXTENSION_INCHES, AbeConstants.SLIDE_SPOOL_CIRCUMFERENCE_INCHES, AbeConstants.SLIDE_EXTENSION_FACTOR);

		telemetry.addData("Status", "Initialized");
		telemetry.update();

		waitForStart();

		slides.extend(16, 32);

		while(opModeIsActive()){
			telemetry.addData("slides", slides.getRelativeExtension());
			telemetry.update();
		}
	}
}
