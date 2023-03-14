package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.abe.AbeConfig;
import org.firstinspires.ftc.teamcode.lib.abe.AbeConstants;
import org.firstinspires.ftc.teamcode.lib.motion.LinearSlides;
import org.firstinspires.ftc.teamcode.lib.motion.PositionableMotor;

@Config
@TeleOp(group = "Tests")
public class SlidesTest extends LinearOpMode {
	public static double LENGTH_TEST = 36.0;

	@Override
	public void runOpMode() throws InterruptedException {
		AbeConfig.Hardware hardware = AbeConfig.loadHardware(hardwareMap);

		PositionableMotor slidesMotor = new PositionableMotor(hardware.slidesMotor, AbeConstants.SLIDES_GEAR_RATIO, AbeConstants.SLIDES_TPR);

		LinearSlides slides = new LinearSlides(slidesMotor, AbeConstants.SLIDES_BASE_LENGTH_INCHES, AbeConstants.SLIDES_BASE_LENGTH_INCHES + AbeConstants.SLIDES_MAX_EXTENSION_INCHES, AbeConstants.SLIDES_SPOOL_CIRCUMFERENCE_INCHES, 1.0);

		telemetry.addData("Status", "Initialized");
		telemetry.update();

		waitForStart();

		double desiredLength;

		while(!isStopRequested()){
			if(gamepad2.a){
				desiredLength = LENGTH_TEST;
			} else {
				desiredLength = slides.getBaseExtension() + 1.0;
			}

			slides.extendTo(desiredLength, AbeConstants.SLIDES_VELOCITY_INCHES);

			telemetry.addData("length", slides.getExtension());
			telemetry.addData("desired length", desiredLength);
			telemetry.update();
		}
	}
}
