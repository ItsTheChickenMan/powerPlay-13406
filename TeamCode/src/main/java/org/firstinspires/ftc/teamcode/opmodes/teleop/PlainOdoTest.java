package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.lib.utils.AngleHelper;
import org.firstinspires.ftc.teamcode.util.Encoder;

@TeleOp(group = "Tests")
public class PlainOdoTest extends LinearOpMode {
	@Override
	public void runOpMode() throws InterruptedException {
		StandardTrackingWheelLocalizer localizer = new StandardTrackingWheelLocalizer(hardwareMap);

		// load motors
		DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
		DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
		DcMotorEx backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
		DcMotorEx backRight = hardwareMap.get(DcMotorEx.class, "backRight");

		frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

		frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

		frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
		backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

		// load encoders
		Encoder leftEncoder = new Encoder(frontLeft);
		Encoder rightEncoder = new Encoder(backRight);
		Encoder frontEncoder = new Encoder(frontRight);

		leftEncoder.setDirection(Encoder.Direction.REVERSE);

		telemetry.addData("Status", "Initialized");
		telemetry.update();

		waitForStart();

		while(!isStopRequested()){
			localizer.update();

			double forward = -gamepad1.left_stick_y;
			double strafe = gamepad1.left_stick_x;
			double rotate = gamepad1.right_stick_x;

			double fl = forward + strafe + rotate;
			double fr = forward - strafe - rotate;
			double bl = forward - strafe + rotate;
			double br = forward + strafe - rotate;

			frontLeft.setPower(fl);
			frontRight.setPower(fr);
			backLeft.setPower(bl);
			backRight.setPower(br);

			double leftEncoderPosition = leftEncoder.getCurrentPosition();
			double rightEncoderPosition = rightEncoder.getCurrentPosition();

			double leftEncoderPositionInches = StandardTrackingWheelLocalizer.encoderTicksToInches(leftEncoderPosition);
			double rightEncoderPositionInches = StandardTrackingWheelLocalizer.encoderTicksToInches(rightEncoderPosition);
			//double frontEncoderPositionInches = StandardTrackingWheelLocalizer.encoderTicksToInches(frontEncoder.getCurrentPosition());

			double correctedLeftEncoderPositionInches = StandardTrackingWheelLocalizer.LEFT_X_MULTIPLIER * leftEncoderPositionInches;
			double correctedRightEncoderPositionInches = StandardTrackingWheelLocalizer.RIGHT_X_MULTIPLIER * rightEncoderPositionInches;

			/*
			double leftEncoderVelocity = leftEncoder.getVelocity();
			double rightEncoderVelocity = rightEncoder.getVelocity();
			double frontEncoderVelocity = frontEncoder.getVelocity();
			 */

			double heading = (correctedLeftEncoderPositionInches - correctedRightEncoderPositionInches) / StandardTrackingWheelLocalizer.LATERAL_DISTANCE;

			heading *= -1;

			telemetry.addData("heading (degrees)", Math.toDegrees(AngleHelper.normalizeAngleRadians(heading)));
			telemetry.addData("rr heading (degrees)", Math.toDegrees(AngleHelper.normalizeAngleRadians(localizer.getPoseEstimate().getHeading())));
			telemetry.addData("corrected left enc position inches", correctedLeftEncoderPositionInches);
			telemetry.addData("corrected right enc position inches", correctedRightEncoderPositionInches);
			telemetry.addData("left enc position inches", leftEncoderPositionInches);
			telemetry.addData("right enc position inches", rightEncoderPositionInches);
			telemetry.addData("left encoder rotations", leftEncoderPosition / StandardTrackingWheelLocalizer.TICKS_PER_REV);
			telemetry.addData("right encoder rotations", rightEncoderPosition / StandardTrackingWheelLocalizer.TICKS_PER_REV);
			telemetry.update();
		}
	}
}
