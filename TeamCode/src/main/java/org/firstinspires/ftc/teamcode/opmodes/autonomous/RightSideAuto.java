package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import android.provider.Settings;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.firstinspires.ftc.teamcode.lib.abe.AbeAutonomous;
import org.firstinspires.ftc.teamcode.lib.abe.AbeConstants;
import org.firstinspires.ftc.teamcode.lib.abe.AbeDrive;
import org.firstinspires.ftc.teamcode.lib.utils.AprilTagDetector;
import org.firstinspires.ftc.teamcode.lib.utils.GlobalStorage;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class RightSideAuto extends AbeAutonomous {
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
		Vector2D depositJunction = new Vector2D(72, 48);

		setup(camera, Mode.RIGHT, depositJunction);

		// load current state from global storage
		this.loadStateFromGlobalStorage();

		// set start point
		setStartPoint(9.875, 33, 0);

		// open fingers
		this.abe.arm.unclampFingers();

		Vector2d depositPosition = new Vector2d(57, 33);

		Pose2d depositPose = new Pose2d(depositPosition.getX(), depositPosition.getY(),
						this.abe.drive.calculateAimAngle(
										AbeDrive.apacheVectorToRRVector(depositJunction).minus(depositPosition)
						)
		);

		// generate opening trajectory (others are generated after depositing, not the best but it's needed to ensure accuracy)
		// (maybe in the future have some homebrew pathfinding?)
		Trajectory openingTrajectory = this.abe.drive.trajectoryBuilder(this.abe.drive.getPoseEstimate())
						// overshoot to move cone
						.splineToConstantHeading(new Vector2d(depositPosition.getX()+3, depositPosition.getY()), 0)
						.splineToConstantHeading(depositPosition, 0)
						.build();

		//
		Trajectory[] parkingTrajectories = new Trajectory[3];

		for(int i = 0; i < 3; i++){
			parkingTrajectories[i] = this.abe.drive.trajectoryBuilder(openingTrajectory.end())
							.splineToConstantHeading(AbeDrive.apacheVectorToRRVector(this.getParkingSpot(i)), 0)
							.build();
		}

		// calculate initial elbow angle
		double openingElbowAngle = this.abe.arm.calculateAimElbowAngleRadians(depositJunction.getX(), depositPose.getY());

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

		camera.closeCameraDeviceAsync(new OpenCvCamera.AsyncCameraCloseListener() {
			@Override
			public void onClose() {

			}
		});

		// check for validity
		if(!AprilTagDetector.isIdValid(aprilTagDetection)){
			// 1 requires the least movement to park
			aprilTagDetection = 1;
		}

		// amount of cones we should attempt
		int coneAttempts = 2;

		// subtract one if parking space is too far
		/*if(aprilTagDetection == 0 || aprilTagDetection == 2){
			coneAttempts--;
		}*/

		// set elbow angle to approximate correct angle (shaves off a bit of time)
		this.abe.arm.enableManualControl();
		this.abe.arm.setElbowAngleRadians(openingElbowAngle, Math.toRadians(20));
		this.abe.arm.disableManualControl();

		// move to position
		this.abe.drive.followTrajectory(openingTrajectory);

		// set appropriate cycle state
		// (we already have a cone, so skip to aim state to ensure that we're properly aiming to the junction)
		this.setCycleState(CycleState.AIMING);

		// cycle cones in stack
		while(this.getConesInStack() >= (5-coneAttempts)){
			cycle();

			update();
		}

		this.abe.arm.enableManualControl();

		// retract slides
		this.abe.arm.extendSlidesTo(this.abe.arm.getSlidesBaseExtension(), 15.0);

		// retract elbow
		this.abe.arm.setElbowAngleDegrees(25.0, 25.0);

		this.abe.arm.disableManualControl();

		Trajectory parkingTrajectory = parkingTrajectories[aprilTagDetection];

		// follow trajectory
		this.abe.drive.followTrajectory(parkingTrajectory);

		// actively save state
		this.saveStateToGlobalStorage();

		// save state
		while(opModeIsActive()){
			// update robot
			this.abe.drive.updateRROnly();
		}
	}
}
