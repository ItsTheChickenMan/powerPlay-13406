package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.lib.abe.AbeAutonomous;
import org.firstinspires.ftc.teamcode.lib.abe.AbeConstants;
import org.firstinspires.ftc.teamcode.lib.abe.AbeTeleOp;
import org.firstinspires.ftc.teamcode.lib.utils.GlobalStorage;
import org.firstinspires.ftc.teamcode.lib.utils.PIDControllerRotation;
import org.firstinspires.ftc.teamcode.opmodes.teleop.AbeAutomatic;

/**
 * @brief Generic autonomous program with various settings (not really meant for actual use, but could be used?)
 */
@Config
public class GenericAuto extends AbeAutonomous {
	/**
	 * Maximum amount of time the auto will run before parking no matter what.  Helps ensure that bad failures or plain slowness don't prevent the auto from ending and screwing up global storage, and ensures the park points are gotten
	 */
	public static double MAXIMUM_TIME_BEFORE_PARK = 28; // set to <=0 to never park until all cones are cycled (mainly just for debugging)
	public static double TELEMETRY_UPDATE_RATE_MS = 50;

	// settings... //
	public Mode MODE = Mode.RIGHT; // mode
	public Vector2d DEPOSIT_POSITION = new Vector2d(56.5, 34.5); // place where the robot should be while depositing
	public int[] DEPOSIT_JUNCTION = {3, 2}; // junction to deposit on using junction coords
	public double SIGNAL_PUSH_DISTANCE_INCHES = 1.0; // distance to push the signal (doesn't tend to line up with actual distance bc splines)
	public int[] DEPOSIT_ATTEMPTS = {6, 6, 6}; // amount of deposit attempts to make depending on randomization, preload included (1 being preload only)
	public PIDControllerRotation.RotationDirection ROTATION_DIRECTION = PIDControllerRotation.RotationDirection.FASTEST;
	public SampleMecanumDrive.LocalizationType LOCALIZATION_TYPE = SampleMecanumDrive.LocalizationType.THREE_WHEEL;

	/**
	 * @brief Called at the start of the autonomous init to override any values of settings.  override this method for each autonomous
	 */
	public void settings(){}

	@Override
	public void runOpMode() throws InterruptedException {
		setAutoBulkReads();

		telemetry.setMsTransmissionInterval((int)TELEMETRY_UPDATE_RATE_MS);

		GlobalStorage.globalTelemetry = telemetry;

		this.settings();

		// use three wheel for auto for best position accuracy during repeated rotations (heading drift not a huge issue here?)
		setup(MODE, DEPOSIT_JUNCTION, LOCALIZATION_TYPE);

		// set rotation direction
		this.abe.drive.setRotationDirection(ROTATION_DIRECTION);

		// load state from global storage
		this.loadStateFromGlobalStorage();

		// determine all mode based values
		Pose2d startPose = new Pose2d(getPositionAccordingToMode(AbeAutomatic.STARTING_POSE.vec()), AbeAutomatic.STARTING_POSE.getHeading());

		Vector2d depositJunctionPosition = getDepositJunctionPosition();
		Vector2d depositPosition = getPositionAccordingToMode(DEPOSIT_POSITION);

		// set start position
		setPoseEstimate(startPose);

		// generate trajectories

		// calculate angle at end of trajectory
		double depositAngle = this.abe.drive.calculateAimAngle(depositJunctionPosition.minus(depositPosition));

		Trajectory toDepositPositionTrajectory;

		/*if(SIGNAL_PUSH_DISTANCE_INCHES < 0.01){
			toDepositPositionTrajectory = this.abe.drive.trajectoryBuilder(startPose)
							.splineToSplineHeading(new Pose2d(depositPosition, depositAngle), 0)
							.build();
		} else {*/
		if(SIGNAL_PUSH_DISTANCE_INCHES < 0.01) SIGNAL_PUSH_DISTANCE_INCHES = 0.01; // prevents dumb exception

		toDepositPositionTrajectory = this.abe.drive.trajectoryBuilder(startPose)
							.splineToSplineHeading(new Pose2d(depositPosition.getX() + SIGNAL_PUSH_DISTANCE_INCHES, depositPosition.getY(), 0), 0)
							.splineToSplineHeading(new Pose2d(depositPosition, depositAngle), 0)
							.build();
		//}

		Trajectory[] parkingTrajectories = new Trajectory[3];

		for(int i = 0; i < 3; i++){
			Vector2d spot = getParkingSpot(i);

			parkingTrajectories[i] = this.abe.drive.trajectoryBuilder(toDepositPositionTrajectory.end())
							.lineToLinearHeading(new Pose2d(spot, 0))
							.build();
		}

		this.abe.arm.setHandUnclamped();

		// search for tags
		int spottedTagId = 1;
		boolean found = false;

		while(!isStarted()){
			// clamp
			if(gamepad1.a || gamepad2.a){
				this.abe.arm.setHandClamped();
			}

			// look for april tags
			this.signalSleeveDetector.search();

			telemetry.addData("Status", "Initialized");

			if(this.signalSleeveDetector.hasTag()){
				spottedTagId = this.signalSleeveDetector.getSpottedTagId();

				telemetry.addData("April Tag found!  Tag id", spottedTagId);
			}

			telemetry.update();
		}

		// main... //

		ElapsedTime autoTimer = new ElapsedTime();

		GlobalStorage.didAuto = true;

		// close camera
		closeCameraAsync();

		int depositAttempts = DEPOSIT_ATTEMPTS[spottedTagId];

		// bring arm to good position
		this.abe.clearAim();

		this.abe.arm.setElbowAngleDegrees(51.0);

		// bring wrist up
		this.abe.arm.setDesiredWristAngleDegrees(AbeConstants.WRIST_HIGH_ANGLE_DEGREES);

		this.abe.arm.update(true, false);

		// roll out to deposit position
		this.abe.drive.followTrajectory(toDepositPositionTrajectory);

		// update drive with imu while we're not moving
		correctPoseEstimateWithIMU();

		double startTime = getGlobalTimeSeconds();
		double duration = 1;
		int updates = 0;

		// depositing some cones?
		if(depositAttempts > 0) {
			setManualBulkReads();

			// auto cycles
			while(this.getConesInStack() > (CONES_IN_STACK_AT_START - depositAttempts + 1) && (MAXIMUM_TIME_BEFORE_PARK <= 0 || autoTimer.seconds() < MAXIMUM_TIME_BEFORE_PARK) && !isStopRequested()){
				clearBulkCache();

				cycle();

				update();

				/*telemetry.addData("pose estimate", this.abe.drive.getPoseEstimate());
				telemetry.addData("drive steady?", this.abe.drive.isSteady());
				telemetry.addData("elbow steady?", this.abe.arm.isElbowSteady());
				telemetry.addData("slides steady?", this.abe.arm.areSlidesSteady());
				telemetry.addData("elbow error", this.abe.arm.getElbowAngleErrorDegrees());
				telemetry.addData("drive error", this.abe.drive.getAimErrorDegrees());
				telemetry.update();*/

				telemetry.addData("pose estimate", this.abe.drive.getPoseEstimate());
				/*telemetry.addData("slides error at time", clampTime);
				telemetry.addData("updates at clamp time", aaa);
				telemetry.addData("updates since cycle switch", updatesSinceCycleSwitch);
				telemetry.addData("time", getGlobalTimeSeconds());*/
				telemetry.addData("drive steady?", this.abe.drive.isSteady());
				telemetry.addData("elbow steady?", this.abe.arm.isElbowSteady());
				telemetry.addData("slides steady?", this.abe.arm.areSlidesSteady());
				telemetry.addData("drive error", this.abe.drive.getAimErrorDegrees());

				telemetry.update();

				updates++;
			}

			duration = getGlobalTimeSeconds() - startTime;

			setAutoBulkReads();
		}

		// move arm to neutral position
		this.abe.clearAim();

		//this.abe.arm.aimAt(AbeTeleOp.ARM_DEFAULT_POSITION_INCHES.getX(), AbeTeleOp.ARM_DEFAULT_POSITION_INCHES.getY());
		this.abe.arm.setElbowAngleRadians(AbeConstants.ELBOW_INITIALIZATION_ANGLE_RADIANS);
		this.abe.arm.setSlidesLengthInches(this.abe.arm.getSlidesBaseExtension() + 1.0);

		// bring wrist up
		this.abe.arm.setDesiredWristAngleDegrees(AbeAutonomous.WRIST_LIFTING_ANGLE_DEGREES);
		//this.abe.arm.setDesiredWristAngleDegrees(AbeConstants.WRIST_DEFAULT_ANGLE_DEGREES);

		this.abe.arm.update(true, true);

		// park
		Trajectory parkTrajectory = parkingTrajectories[spottedTagId];

		this.abe.drive.followTrajectoryAsync(parkTrajectory);

		// save at least once
		this.saveStateCorrectPose();

		while(!isStopRequested()){
			// save state
			// without this additional check, the whole thing breaks.  I wish I knew why...
			// don't save in the last 1/10th of a second because saving at the end of the opmode breaks the save?
			if(autoTimer.seconds() < 29.9){
				this.saveStateCorrectPose();
			}

			// update rr only
			this.abe.drive.updateRoadrunnerDrive();

			telemetry.addData("avg loop time (ms)", (duration / updates) * 1000);
			telemetry.addData("loops/second", (double)updates / duration);
			telemetry.update();
		}
	}

	/**
	 * @brief Saves the state, but uses the imu for pose rather than the given value to ensure there's no drift
	 *
	 * @todo can this be worked into a self-correcting three wheel odometry system more reliable than just three wheel?
	 */
	public void saveStateCorrectPose(){
		// save state
		this.saveStateToGlobalStorage();

		// override pose estimate
		Pose2d corrected = new Pose2d(
						// get just position values
						this.abe.drive.getPoseEstimate().vec(),

						// use imu for heading
						this.abe.drive.getRawExternalHeading()
		);

		// override saved estimate
		GlobalStorage.currentPose = corrected;
	}

	public void correctJunctionOffset(){
		// add drift
		GlobalStorage.autoJunctionOffset = GlobalStorage.autoJunctionOffset.plus(new Vector2d(JUNCTION_OFFSET_DRIFT_X, JUNCTION_OFFSET_DRIFT_Y));
	}
}