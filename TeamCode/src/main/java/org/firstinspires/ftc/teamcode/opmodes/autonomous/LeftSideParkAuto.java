package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.firstinspires.ftc.teamcode.lib.abe.AbeAutonomous;
import org.firstinspires.ftc.teamcode.lib.abe.AbeDrive;
import org.firstinspires.ftc.teamcode.lib.utils.AprilTagDetector;
import org.firstinspires.ftc.teamcode.lib.utils.GlobalStorage;
import org.firstinspires.ftc.teamcode.lib.utils.JunctionHelper;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class LeftSideParkAuto extends AbeAutonomous {
	@Override
	public void runOpMode() throws InterruptedException {
		GlobalStorage.globalTelemetry = telemetry;
		GlobalStorage.didAuto = true;

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
		Vector2D depositJunction = new Vector2D(72, JunctionHelper.FIELD_WIDTH - 48);

		setup(camera, Mode.LEFT, depositJunction);

		// load current state from global storage
		this.loadStateFromGlobalStorage();

		// set start point
		setStartPoint(9.875, JunctionHelper.FIELD_WIDTH - 33, 0);

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

		camera.closeCameraDeviceAsync(new OpenCvCamera.AsyncCameraCloseListener() {
			@Override
			public void onClose() {

			}
		});

		// check april tag detection
		int aprilTagDetection = this.aprilTagDetector.getSpottedTagId();

		// check for validity
		if(!AprilTagDetector.isIdValid(aprilTagDetection)){
			// 1 requires the least movement to park
			aprilTagDetection = 1;
		}

		// park trajectory
		Trajectory driveTrajectory = this.abe.drive.trajectoryBuilder(this.abe.drive.getPoseEstimate())
						.splineToConstantHeading(new Vector2d(54.0, JunctionHelper.FIELD_WIDTH - 33), 0)
						.build();

		Trajectory parkTrajectory = this.abe.drive.trajectoryBuilder(driveTrajectory.end())
						.splineToConstantHeading(AbeDrive.apacheVectorToRRVector(this.getParkingSpot(aprilTagDetection)), 0)
						.build();

		this.abe.drive.followTrajectory(driveTrajectory);
		this.abe.drive.followTrajectory(parkTrajectory);

		this.saveStateToGlobalStorage();

		while(!isStopRequested()) {
			telemetry.addLine("Global Storage:");
			GlobalStorage.logGlobalStorage(telemetry);

			telemetry.update();
		}
	}
}
