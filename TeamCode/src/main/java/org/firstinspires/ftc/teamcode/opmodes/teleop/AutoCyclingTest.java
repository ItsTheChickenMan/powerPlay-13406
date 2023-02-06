package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.firstinspires.ftc.teamcode.lib.abe.AbeAutonomous;
import org.firstinspires.ftc.teamcode.lib.abe.AbeDrive;
import org.firstinspires.ftc.teamcode.lib.utils.AprilTagDetector;
import org.firstinspires.ftc.teamcode.lib.utils.GlobalStorage;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

// simple test for the cycle from cone stack to a junction (either tall or medium, whichever is statistically more reliable)
@TeleOp(group = "Tests")
public class AutoCyclingTest extends AbeAutonomous {
	// note: not including preload
	protected int[] depositAttempts = new int[]{5, 5, 5};
	protected Vector2D depositJunction = new Vector2D(72, 48);

	public void settings(){
	}

	@Override
	public void runOpMode() throws InterruptedException {
		GlobalStorage.globalTelemetry = telemetry;
		GlobalStorage.didAuto = true;

		this.settings();

		// setup auto
		setup(null, Mode.RIGHT, depositJunction);

		// load current state from global storage
		this.loadStateFromGlobalStorage();

		// open fingers
		this.abe.arm.unclampFingers();

		Vector2d depositPosition = new Vector2d(55, 33);

		// set start point
		setStartPoint(depositPosition.getX(), depositPosition.getY(), 0);

		// calculate initial elbow angle
		//double openingElbowAngle = this.abe.arm.calculateAimElbowAngleRadians(depositJunction.getX(), depositJunction.getY());
		double openingElbowAngle = 45;

		// wait for start
		while (!isStarted() && !isStopRequested()) {
			// clamp when user presses a
			if (gamepad1.a || gamepad2.a) {
				this.abe.arm.clampFingers();
			}

			// telemetry
			telemetry.addData("Status", "Initialized");
			telemetry.update();
		}

		// START

		// amount of cones we should attempt
		int coneAttempts = this.depositAttempts[1];

		// subtract one if parking space is too far
		/*if(aprilTagDetection == 0 || aprilTagDetection == 2){
			coneAttempts--;
		}*/

		// set appropriate cycle state
		// (we already have a cone, so skip to aim state to ensure that we're properly aiming to the junction)
		this.setCycleState(CycleState.AIMING);

		int totalLoops = 0;

		double start = this.timer.seconds();

		// cycle cones in stack
		while (this.getConesInStack() >= (5 - coneAttempts)) {
			cycle();

			update();

			telemetry.addData("velocity", this.abe.drive.drive.getPoseVelocity());
			GlobalStorage.globalTelemetry.update();

			totalLoops++;
		}

		double end = this.timer.seconds();

		double duration = end - start;
		double averageLoopTime = duration / totalLoops;

		while (opModeIsActive()) {
			this.abe.drive.drive.updatePoseEstimate();

			telemetry.addData("total loops", totalLoops);
			telemetry.addData("duration", duration);
			telemetry.addData("avg time (milliseconds)", averageLoopTime * 1000);
			telemetry.update();
		}
	}
}
