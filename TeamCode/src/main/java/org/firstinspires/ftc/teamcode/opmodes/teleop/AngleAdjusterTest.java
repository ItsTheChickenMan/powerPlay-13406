package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.lib.abe.AbeArm;
import org.firstinspires.ftc.teamcode.lib.abe.AbeConstants;
import org.firstinspires.ftc.teamcode.lib.motion.AngleAdjuster;

@TeleOp(group = "Tests")
public class AngleAdjusterTest extends LinearOpMode {

	@Override
	public void runOpMode() throws InterruptedException {
		DcMotorEx elbowMotor = hardwareMap.get(DcMotorEx.class, "angleMotor");
		TouchSensor limitSwitch = hardwareMap.get(TouchSensor.class, "limitSwitch");

		elbowMotor.setDirection(DcMotorSimple.Direction.REVERSE);

		AngleAdjuster elbow = new AngleAdjuster(elbowMotor, AbeConstants.ELBOW_GEAR_RATIO, AbeConstants.ELBOW_TICK_RATIO, AbeConstants.ELBOW_LOWER_LIMIT_ROTATIONS, AbeConstants.ELBOW_UPPER_LIMIT_ROTATIONS, limitSwitch);

		telemetry.addData("Status", "Initialized");
		telemetry.update();

		waitForStart();

		while(!isStopRequested()){
			elbow.rotateToRadians(Math.toRadians(45.0), Math.toRadians(20.0));

			telemetry.addData("elbowAngle", elbow.getAngleDegrees());
			telemetry.update();
		}
	}
}