package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.lib.utils.AngleHelper;
import org.firstinspires.ftc.teamcode.lib.utils.Vec2;

/**
 * @brief Test opmode for math involving the auto rotate
 */
@TeleOp
public class RotateToPointTest extends LinearOpMode {
	@Override
	public void runOpMode() throws InterruptedException {
		double armOffset = 2;

		SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

		drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

		waitForStart();

		double pointAtX = 0;
		double pointAtY = 0;

		boolean pointingEnabled = false;

		double lastError = 0;

		while(!isStopRequested()){
			pointingEnabled = pointingEnabled || gamepad1.a;

			// update odometry
			drive.update();

			Pose2d poseEstimate = drive.getPoseEstimate();

			// get heading
			double heading = poseEstimate.getHeading();

			// drive
			double forward = -gamepad1.left_stick_y;
			double strafe = gamepad1.left_stick_x;
			//double rotate = gamepad1.right_stick_x;
			double slow = Math.min(1.2 - Math.max(gamepad1.right_trigger, gamepad1.left_trigger), 1.0);

			forward *= slow;
			strafe *= slow;

			// rotate movement vector
			Vec2 driveVec = new Vec2(strafe, forward);

			driveVec.rotate(-heading);

			forward = driveVec.y;
			strafe = driveVec.x;

			double offsetX = poseEstimate.getX() - pointAtX;
			double offsetY = poseEstimate.getY() - pointAtY;
			double distance = Math.sqrt(offsetX*offsetX + offsetY*offsetY);

			double desiredAngle = Math.atan2(offsetY, offsetX) + Math.asin(armOffset / distance);
			desiredAngle += Math.PI;

			double p = 1.5;
			double d = 2;

			double err = AngleHelper.angularDistanceRadians(desiredAngle, heading);
			double roc = Math.abs(err - lastError);

			lastError = err;

			double rotate = err*p - Math.max(roc*d, 0);

			rotate = Math.min(Math.max(rotate, -0.75), 0.75);

			telemetry.addData("rotate", rotate);

			if(!pointingEnabled) rotate = 0;

			// set powers
			double frontLeft = forward + strafe + rotate;
			double frontRight = forward - strafe - rotate;
			double backLeft = forward - strafe + rotate;
			double backRight = forward + strafe - rotate;

			// normalize
			double largest = Math.max(backRight, Math.max(backLeft, Math.max(frontLeft, frontRight)));

			if(largest > 1.0){
				frontLeft /= largest;
				frontRight /= largest;
				backLeft /= largest;
				backRight /= largest;
			}

			// logical!
			drive.setMotorPowers(frontLeft, backLeft, backRight, frontRight);

			//telemetry.addData("desired angle", Math.toDegrees(desiredAngle));
			telemetry.addData("heading", Math.toDegrees(heading));
			//telemetry.addData("x", poseEstimate.getX());
			//telemetry.addData("y", poseEstimate.getY());
			telemetry.update();
		}
	}
}
