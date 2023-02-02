package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.firstinspires.ftc.teamcode.lib.abe.AbeAutonomous;
import org.firstinspires.ftc.teamcode.lib.abe.AbeDrive;
import org.firstinspires.ftc.teamcode.lib.utils.AprilTagDetector;
import org.firstinspires.ftc.teamcode.lib.utils.GlobalStorage;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class GenericRightSideAuto extends AbeAutonomous {
	// note: not including preload
	protected int[] depositAttempts = new int[]{2, 3, 2};
	protected Vector2D depositJunction = new Vector2D(72, 48);

	public void settings(){
		this.depositAttempts = new int[]{2, 3, 2};
		this.depositJunction = new Vector2D(72, 48);
	}

	@Override
	public void runOpMode() throws InterruptedException {
		GlobalStorage.globalTelemetry = telemetry;
		GlobalStorage.didAuto = true;

		this.settings();

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
		setup(camera, Mode.RIGHT, depositJunction);

		// load current state from global storage
		this.loadStateFromGlobalStorage();

		// set start point
		setStartPoint(10.25, 33, 0);

		// open fingers
		this.abe.arm.unclampFingers();

		Vector2d depositPosition = new Vector2d(55, 33);

		Pose2d depositPose = new Pose2d(depositPosition.getX(), depositPosition.getY(),
						this.abe.drive.calculateAimAngle(
										AbeDrive.apacheVectorToRRVector(depositJunction).minus(depositPosition)
						)
		);

		// generate opening trajectory (others are generated after depositing, not the best but it's needed to ensure accuracy)
		// (maybe in the future have some homebrew pathfinding?)
		Trajectory openingTrajectory = this.abe.drive.trajectoryBuilder(this.abe.drive.getPoseEstimate())
						// overshoot to move cone
						.splineToConstantHeading(new Vector2d(depositPosition.getX()+5, depositPosition.getY()), 0)
						.splineToSplineHeading(depositPose, 0)
						.build();

		// generate park trajectories
		//Trajectory[] parkingTrajectories = new Trajectory[3];

		/*for(int i = 0; i < 3; i++){
			parkingTrajectories[i] = this.abe.drive.trajectoryBuilder(openingTrajectory.end())
							.splineToSplineHeading(new Pose2d(this.getParkingSpot(i).getX(), this.getParkingSpot(i).getY(), 0), 0)
							.build();
		}*/

		// calculate initial elbow angle
		//double openingElbowAngle = this.abe.arm.calculateAimElbowAngleRadians(depositJunction.getX(), depositJunction.getY());
		double openingElbowAngle = 45;

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
		int coneAttempts = this.depositAttempts[aprilTagDetection];

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

		int totalLoops = 0;

		double start = this.timer.seconds();

		// cycle cones in stack
		while(this.getConesInStack() >= (5-coneAttempts)){
			cycle();

			update();

			GlobalStorage.globalTelemetry.update();

			totalLoops++;
		}

		double end = this.timer.seconds();

		double duration = end - start;
		double averageLoopTime = duration / totalLoops;

		while(opModeIsActive()){
			telemetry.addData("total loops", totalLoops);
			telemetry.addData("duration", duration);
			telemetry.addData("avg time (milliseconds)", averageLoopTime*1000);
			telemetry.update();
		}

		this.abe.clearPoint();

		this.abe.update();

		this.abe.arm.aimAt(6, 20);

		this.abe.update();

		/*while(opModeIsActive()){
			this.abe.update();

			telemetry.addData("pose", this.abe.drive.getPoseEstimate());
			telemetry.update();
		}*/
		//Trajectory parkingTrajectory = parkingTrajectories[aprilTagDetection];
		Trajectory parkingTrajectory = this.abe.drive.trajectoryBuilder(this.abe.drive.getPoseEstimate())
						.splineToLinearHeading(new Pose2d(this.getParkingSpot(aprilTagDetection).getX(), this.getParkingSpot(aprilTagDetection).getY(), 0), 0)
						.build();

		// follow trajectory
		this.abe.drive.followTrajectory(parkingTrajectory);

		// save state
		this.saveStateToGlobalStorage();

		while(opModeIsActive()){
			// update robot
			this.abe.drive.updateRROnly();
		}
	}
}
