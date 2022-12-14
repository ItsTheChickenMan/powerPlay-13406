package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.firstinspires.ftc.teamcode.lib.abe.AbeAutonomous;
import org.firstinspires.ftc.teamcode.lib.utils.AprilTagDetector;
import org.firstinspires.ftc.teamcode.lib.utils.GlobalStorage;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class RightSideAuto extends AbeAutonomous {
	@Override
	public void runOpMode() throws InterruptedException {
		GlobalStorage.globalTelemetry = telemetry;

		OpenCvCamera camera = AbeAutonomous.createCamera(hardwareMap);

		camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
		{
			@Override
			public void onOpened()
			{
				camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
			}

			@Override
			public void onError(int errorCode)
			{

			}
		});

		// setup auto
		setup(camera, Mode.RIGHT, new Vector2D(72, 48));

		// set start point
		//setStartPoint(55.5, 34.5, 0);
		setStartPoint(0, 0, 0);

		// open fingers
		this.abe.arm.unclampFingers();

		// wait for start
		while(!isStarted() && !isStopRequested()){
			// clamp when user presses a
			if(gamepad1.a || gamepad2.a){
				this.abe.arm.clampFingers();
			}

			// search for april tags
			this.aprilTagDetector.search();

			// telemetry
			telemetry.addData("Status", "Initialized");

			if(this.aprilTagDetector.hasTag()){
				telemetry.addLine("April Tag found!  Tag id: " + this.aprilTagDetector.getSpottedTagId());
			}

			telemetry.update();
		}

		// START

		// check april tag detection
		int aprilTagDetection = this.aprilTagDetector.getSpottedTagId();

		// check for validity
		if(!AprilTagDetector.isIdValid(aprilTagDetection)){
			// 1 requires the least movement to park
			aprilTagDetection = 1;
		}

		// amount of cones we should attempt
		int coneAttempts = 5;

		// subtract one if parking space is too far
		if(aprilTagDetection == 0 || aprilTagDetection == 2){
			coneAttempts--;
		}

		/*telemetry.addData("aprilTagDetection", aprilTagDetection);
		telemetry.addData("coneAttempts", coneAttempts);
		telemetry.update();*/

		// move to position

		while(!isStopRequested() && opModeIsActive());

		// cycle cones in stack
		/*while(this.getConesInStack() > (5-coneAttempts)){
			cycle();

			update();

			telemetry.update();
		}*/

		// park...
	}
}
