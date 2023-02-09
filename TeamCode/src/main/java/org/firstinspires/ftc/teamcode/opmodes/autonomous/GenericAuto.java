package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.firstinspires.ftc.teamcode.lib.abe.AbeAutonomous;
import org.firstinspires.ftc.teamcode.lib.abe.AbeDrive;
import org.firstinspires.ftc.teamcode.lib.utils.AprilTagDetector;
import org.firstinspires.ftc.teamcode.lib.utils.GlobalStorage;
import org.firstinspires.ftc.teamcode.lib.utils.JunctionHelper;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

public abstract class GenericAuto extends AbeAutonomous {
	// note: including preload
	protected int[] depositAttempts = new int[]{3, 3, 3};
	protected Mode mode = Mode.RIGHT;

	public static double CYCLE_TIME_REQUIRED = 6;

	// NOTE: always relative to right side, mirrors automatically based on mode
	protected Vector2D depositJunction = new Vector2D(72, 47);

	public void settings(){
		this.depositAttempts = new int[]{2, 3, 2};
		this.depositJunction = new Vector2D(72, 47);
		this.mode = Mode.LEFT;
	}

	@Override
	public void runOpMode() throws InterruptedException {
		GlobalStorage.globalTelemetry = telemetry;
		GlobalStorage.didAuto = true;

		this.settings();

		if(this.mode == Mode.LEFT){
			this.depositJunction = new Vector2D(this.depositJunction.getX(), JunctionHelper.FIELD_WIDTH - this.depositJunction.getY());
		}

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
		setup(camera, this.mode, depositJunction);

		// load current state from global storage
		this.loadStateFromGlobalStorage();

		// set start point
		setStartPoint(this.getStartingPosition().getX(), this.getStartingPosition().getY(), 0);

		// open fingers
		this.abe.arm.unclampFingers();

		Vector2d depositPosition = AbeDrive.apacheVectorToRRVector(this.getDepositPosition());

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
						.splineToConstantHeading(new Vector2d(depositPose.getX(), depositPose.getY()), 0)
						.build();

		// generate park trajectories
		Trajectory[] parkingTrajectories = new Trajectory[3];

		for(int i = 0; i < 3; i++){
			parkingTrajectories[i] = this.abe.drive.trajectoryBuilder(openingTrajectory.end())
							.splineToLinearHeading(new Pose2d(this.getParkingSpot(i).getX(), this.getParkingSpot(i).getY(), 0), 0)
							.build();
		}

		// FIXME: calculate initial elbow angle

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

		ElapsedTime autoTimer = new ElapsedTime();
		autoTimer.reset();

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

		if(coneAttempts > 0) {
			// subtract one if parking space is too far
			/*if(aprilTagDetection == 0 || aprilTagDetection == 2){
				coneAttempts--;
			}*/

			// set elbow angle to approximate correct angle (shaves off a bit of time)
			this.abe.arm.aimAt(12, 23);
			this.abe.arm.update();

			// move to position
			this.abe.drive.followTrajectory(openingTrajectory);

			// set appropriate cycle state
			// (we already have a cone, so skip to aim state to ensure that we're properly aiming to the junction)
			this.setCycleState(CycleState.AIMING);

			int totalLoops = 0;

			double start = this.timer.seconds();

			// cycle cones in stack
			// FIXME: add the backup timer back (fix to only check during safe states)
			while (this.getConesInStack() > (5 - coneAttempts) && !isStopRequested()/* && (30 - autoTimer.seconds()) > CYCLE_TIME_REQUIRED*/) {
				cycle();

				update();

				telemetry.addData("elbow steady", this.abe.arm.isElbowSteady());
				telemetry.addData("slides steady", this.abe.arm.isSlidesSteady());
				telemetry.addData("drive steady", this.abe.drive.isSteady());
				GlobalStorage.globalTelemetry.update();

				totalLoops++;
			}

			double end = this.timer.seconds();

			double duration = end - start;
			double averageLoopTime = duration / totalLoops;
		}

		this.abe.clearPoint();

		this.abe.update();

		this.abe.arm.aimAt(6, 20);

		this.abe.update();

		Trajectory parkingTrajectory;

		if((30 - autoTimer.seconds()) < 2){
			// use pregenerated in a pinch
			parkingTrajectory = parkingTrajectories[aprilTagDetection];
		} else {
			parkingTrajectory = this.abe.drive.trajectoryBuilder(this.abe.drive.getPoseEstimate())
							.splineToLinearHeading(new Pose2d(this.getParkingSpot(aprilTagDetection).getX(), this.getParkingSpot(aprilTagDetection).getY(), 0), 0)
							.build();
		}
		//Trajectory parkingTrajectory = parkingTrajectories[aprilTagDetection];

		// follow trajectory
		this.abe.drive.followTrajectoryAsync(parkingTrajectory);

		//this.saveStateToGlobalStorage();


		// amount of time to save for
		double endTime = autoTimer.seconds();
		double saveTime = 2;

		// save at least once
		this.saveStateToGlobalStorage();

		while(!isStopRequested()){
			this.abe.drive.updateRROnly();

			telemetry.addData("angle", this.abe.arm.getElbowAngleDegrees());
			telemetry.addData("length", this.abe.arm.getSlidesExtension());

			// without this additional check, the whole thing breaks.  I wish I knew why...
			// overall, it's generally safe to save for a short amount of time and quit
			// as a just-in-case catch, don't save in the last half second either
			if( (autoTimer.seconds()-endTime) < saveTime && autoTimer.seconds() < 29.5){
				this.saveStateToGlobalStorage();
				telemetry.addData("global angle", GlobalStorage.currentElbowAngleRadians);
				telemetry.addData("global length", GlobalStorage.currentSlidesExtension);
			}

			telemetry.update();
		}
	}
}
