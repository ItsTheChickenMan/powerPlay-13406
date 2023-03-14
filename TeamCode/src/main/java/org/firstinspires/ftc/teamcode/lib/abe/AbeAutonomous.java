package org.firstinspires.ftc.teamcode.lib.abe;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.vuforia.Device;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.lib.utils.GlobalStorage;
import org.firstinspires.ftc.teamcode.lib.utils.SignalSleeveDetector;
import org.firstinspires.ftc.teamcode.lib.utils.JunctionHelper;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
public abstract class AbeAutonomous extends AbeOpMode {
	public static int CONES_IN_STACK_AT_START = 5;

	public static double GRABBING_TIME_SECONDS = 0.25;
	public static double WRISTING_TIME_SECONDS = 0.1;
	public static double WRIST_LIFTING_ANGLE_DEGREES = 105;
	public static double WRIST_LIFTING_TIME_SECONDS = 0.35;

	public static double WRIST_UP_ANGLE_DEGREES = 65.0;
	public static double JUNCTION_HEIGHT_OFFSET_INCHES = 1.75;

	public static double JUNCTION_OFFSET_DRIFT_X = 0.0;
	public static double JUNCTION_OFFSET_DRIFT_Y = -0.5;

	public static double LIFTING_OFFSET_INCHES = 1.0;

	public static PIDCoefficients AIM_HEADING_PID = new PIDCoefficients(5.8, 0, 0.2);

	public static Vector2d[] PARKING_SPOTS_RIGHT = {new Vector2d(57, 60), new Vector2d(57, 30), new Vector2d(57, 8)};

	public enum Mode {
		LEFT,
		RIGHT
	}

	public enum CycleState {
		GRABBING,
		LIFTING,
		AIMING,
		EXTENDING,
		WRISTING,
		DROPPING,
		REORIENTING;

		public static final CycleState[] ORDERED_CYCLE_STATES = {GRABBING, LIFTING, AIMING, EXTENDING, DROPPING, REORIENTING};

		public static CycleState increment(CycleState state){
			CycleState incremented = GRABBING;

			switch(state){
				case GRABBING:
					incremented = LIFTING;
					break;
				case LIFTING:
					incremented = AIMING;
					break;
				case AIMING:
					incremented = EXTENDING;
					break;
				case EXTENDING:
					incremented = WRISTING;
					break;
				case WRISTING:
					incremented = DROPPING;
					break;
				case DROPPING:
					incremented = REORIENTING;
					break;
			}

			return incremented;
		}

		public static CycleState decrement(CycleState state){
			CycleState decremented = REORIENTING;

			switch(state){
				case LIFTING:
					decremented = GRABBING;
					break;
				case AIMING:
					decremented = LIFTING;
					break;
				case EXTENDING:
					decremented = AIMING;
					break;
				case DROPPING:
					decremented = EXTENDING;
					break;
				case WRISTING:
					decremented = DROPPING;
					break;
				case REORIENTING:
					decremented = WRISTING;
					break;
			}

			return decremented;
		}
	}

	protected OpenCvCamera camera;
	protected SignalSleeveDetector signalSleeveDetector;

	private Mode currentMode;
	private CycleState cycleState = CycleState.AIMING;

	private int conesInStack = CONES_IN_STACK_AT_START + 1; // +1 accounts for preload, kinda stupid but it works

	private double stateSwitchScheduler = UNSCHEDULED;
	public int updatesSinceCycleSwitch = 0;
	private double timeOfLastCycleSwitch = 0.0;

	private int[] junctionCoords;

	private boolean doDrive = true;
	private boolean doElbow = true;
	private boolean doSlides = true;

	public CycleState getCycleState(){
		return this.cycleState;
	}

	/**
	 * @brief set up required values for auto
	 */
	public void setup(Mode mode, int[] junctionCoords, SampleMecanumDrive.LocalizationType localizationType){
		initialize(hardwareMap, localizationType, AIM_HEADING_PID);

		this.currentMode = mode;

		this.camera = createCamera("Webcam 1");

		this.openCameraAndStream();

		this.signalSleeveDetector = new SignalSleeveDetector(this.camera);

		this.junctionCoords = junctionCoords;

		this.abe.arm.setSlidesRestingExtensionInches(18.0);
	}

	public double getTimeSinceLastCycleSwitch(){
		return getGlobalTimeSeconds() - this.timeOfLastCycleSwitch;
	}

	public double clampTime = 0.0;
	public int aaa = 0;

	public void cycle(){
		switch(cycleState){
			case GRABBING: {
				// set states
				this.doDrive = true;
				this.doElbow = true;
				this.doSlides = true;

				this.abe.arm.setDesiredWristAngleDegrees(AbeConstants.WRIST_GRAB_ANGLE_DEGREES);

				if(this.updatesSinceCycleSwitch < 5){
					this.abe.arm.setHandUnclamped();
				}

				// aim at stack
				this.abe.arm.setAimWristAngleDegrees(AbeConstants.WRIST_GRAB_ANGLE_DEGREES);
				this.aimAtConeStack();

				if(this.updatesSinceCycleSwitch > 10 && this.abe.isSteady() && !isEventScheduled(this.stateSwitchScheduler)){
					this.abe.arm.setHandClamped();
					this.stateSwitchScheduler = getScheduledTime(GRABBING_TIME_SECONDS + (this.conesInStack == 1 ? WRISTING_TIME_SECONDS : 0));

					clampTime = this.abe.arm.getSlidesLengthErrorInches();
					aaa = this.updatesSinceCycleSwitch;

					// update drive with imu while we're not moving
					//correctPoseEstimateWithIMU();
				}

				if(isEventFiring(this.stateSwitchScheduler) && this.conesInStack == 1){
					// skip lifting state on last cone (increments following the switch)
					this.cycleState = CycleState.LIFTING;
				}

				break;
			}

			case LIFTING: {
				// set states
				this.doDrive = true;
				this.doElbow = true;
				this.doSlides = true;

				this.abe.arm.setHandClamped();
				this.abe.arm.setDesiredWristAngleDegrees(WRIST_LIFTING_ANGLE_DEGREES);

				this.aimAtConeStack(LIFTING_OFFSET_INCHES * (this.currentMode == Mode.RIGHT ? 1.0 : -1.0) );

				if(!isEventScheduled(this.stateSwitchScheduler)){
					this.stateSwitchScheduler = getScheduledTime(WRIST_LIFTING_TIME_SECONDS);
				}

				break;
			}

			case AIMING: {
				// aim to chosen junction
				this.abe.arm.setAimWristAngleDegrees(WRIST_UP_ANGLE_DEGREES);
				this.aimToJunction(junctionCoords[0], junctionCoords[1], GlobalStorage.autoJunctionOffset, JUNCTION_HEIGHT_OFFSET_INCHES);

				// set states
				this.doDrive = true;
				this.doElbow = true;
				this.doSlides = false;

				this.abe.arm.setHandClamped();
				this.abe.arm.setDesiredWristAngleDegrees(WRIST_LIFTING_ANGLE_DEGREES);

				// schedule switch when drive is close enough
				if(this.updatesSinceCycleSwitch > 10 && this.abe.drive.isSteady() && !isEventScheduled(this.stateSwitchScheduler)){
					this.stateSwitchScheduler = getScheduledTimeNow();

					// update drive with imu while we're not moving
					correctPoseEstimateWithIMU();
				}

				break;
			}

			case EXTENDING: {
				// set states
				this.doDrive = true;
				this.doElbow = true;
				this.doSlides = true;

				this.abe.arm.setDesiredWristAngleDegrees(WRIST_UP_ANGLE_DEGREES);
				this.abe.arm.setHandClamped();

				// schedule switch when slides are close enough
				if(/*gamepad2.a && */this.updatesSinceCycleSwitch > 5 && this.abe.isSteady() && !isEventScheduled(this.stateSwitchScheduler)){
					this.stateSwitchScheduler = getScheduledTimeNow();
				}

				break;
			}

			case WRISTING: {
				// set states
				this.doDrive = true;
				this.doElbow = true;
				this.doSlides = true;

				this.abe.arm.setDesiredWristAngleDegrees(AbeConstants.WRIST_DROP_ANGLES_DEGREES[3]);
				this.abe.arm.setHandClamped();

				if(!isEventScheduled(this.stateSwitchScheduler)){
					this.stateSwitchScheduler = getScheduledTime(WRISTING_TIME_SECONDS);
				}

				break;
			}

			case DROPPING: {
				// set states
				this.doDrive = true;
				this.doElbow = true;
				this.doSlides = true;

				this.abe.arm.setDesiredWristAngleDegrees(AbeConstants.WRIST_DROP_ANGLES_DEGREES[3]);
				this.abe.arm.setHandUnclamped();

				if(!isEventScheduled(this.stateSwitchScheduler)){
					this.stateSwitchScheduler = getScheduledTime(AbeTeleOp.AIM_MODE_DEPOSIT_TIME_SECONDS);
				}

				// decrement stack count
				if(isEventFiring(this.stateSwitchScheduler)){
					this.conesInStack--;
				}

				break;
			}

			case REORIENTING: {
				// set states
				this.doDrive = true;
				this.doElbow = true;
				this.doSlides = false;

				// aim to stack
				this.abe.arm.setAimWristAngleDegrees(AbeConstants.WRIST_GRAB_ANGLE_DEGREES);
				this.aimAtConeStack();

				// raise wrist to avoid nabbing the pole
				this.abe.arm.setDesiredWristAngleDegrees(AbeConstants.WRIST_HIGH_ANGLE_DEGREES);
				this.abe.arm.setHandUnclamped();

				/*if(getTimeSinceLastCycleSwitch() > 0.2 && getTimeSinceLastCycleSwitch() < 0.4) {
					this.abe.arm.setHandClamped(); // clamp to avoid catching junction (will probably happen anyways)
				} else {
					this.abe.arm.setHandUnclamped();
				}*/

				// switch when drive + elbow is steady
				if(this.updatesSinceCycleSwitch > 10 && this.abe.drive.isSteady() && this.abe.arm.isElbowSteady() && !isEventScheduled(this.stateSwitchScheduler)){
					this.stateSwitchScheduler = getScheduledTimeNow();
				}

				break;
			}
		}

		//telemetry.addData("cycle state", this.cycleState.toString());

		if(isEventFiring(this.stateSwitchScheduler)){
			// increment cycle state
			this.cycleState = CycleState.increment(this.cycleState);

			// unschedule switch
			this.stateSwitchScheduler = UNSCHEDULED;

			// reset counter
			this.updatesSinceCycleSwitch = 0;

			this.timeOfLastCycleSwitch = getGlobalTimeSeconds();
		}

		this.updatesSinceCycleSwitch++;
	}

	public void update(){
		this.abe.update(doDrive, doElbow, doSlides);
	}

	public Vector2d getDepositJunctionPosition(){
		return getPositionAccordingToMode(JunctionHelper.snappedToRaw(this.junctionCoords));
	}

	/**
	 * @param deviceName
	 * @return a new OpenCvCamera
	 */
	public OpenCvCamera createCamera(String deviceName){
		int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
		return OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, deviceName), cameraMonitorViewId);
	}

	/**
	 * @brief opens the current camera and streams it, asynchronously
	 */
	public void openCameraAndStreamAsync(){
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
	}

	public void openCameraAndStream(){
		camera.openCameraDevice();

		camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
	}

	/**
	 * @brief close the current camera, asynchronously
	 */
	public void closeCameraAsync(){
		camera.closeCameraDeviceAsync(new OpenCvCamera.AsyncCameraCloseListener() {
			@Override
			public void onClose() {
			}
		});
	}

	/**
	 * @param pos coordinates on the right side of the field
	 * @return coordinates on either left or right, corresponding to the value of currentMode
	 */
	public Vector2d getPositionAccordingToMode(Vector2d pos){
		if(this.currentMode == Mode.LEFT){
			return getOppositeCoords(pos);
		}

		return pos;
	}

	/**
	 * @param id
	 * @return the parking position given the randomization id and the current mode
	 */
	public Vector2d getParkingSpot(int id){
		// validate id
		if(id < 0 || id > 2) id = 1; // default to middle?

		// swap id if left
		if(currentMode == Mode.LEFT) id = 2 - id;

		Vector2d spot = PARKING_SPOTS_RIGHT[id];

		return getPositionAccordingToMode(spot);
	}

	public int getConesInStack(){
		return this.conesInStack;
	}

	public Vector2d getConeStackPosition(){
		return getPositionAccordingToMode(getConeStackRightPosition());
	}

	public void aimAtConeStack(){
		aimAtConeStack(0.0);
	}

	public void aimAtConeStack(double offset){
		double height = this.getStackGrabHeight(getConesInStack());

		// determine stack position from mode
		Vector2d stackPosition = this.getConeStackPosition();

		stackPosition = stackPosition.plus(new Vector2d(0, offset));

		// aim
		this.abe.aimAt(stackPosition.getX(), height, stackPosition.getY());
	}
}