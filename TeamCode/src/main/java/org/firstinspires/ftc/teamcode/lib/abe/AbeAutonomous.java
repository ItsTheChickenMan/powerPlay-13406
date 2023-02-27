package org.firstinspires.ftc.teamcode.lib.abe;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

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
	public static double WRIST_LIFTING_ANGLE_DEGREES = 95;
	public static double WRIST_LIFTING_TIME_SECONDS = 0.35;

	public static Vector2d[] PARKING_SPOTS_RIGHT = {new Vector2d(57, 60), new Vector2d(57, 36), new Vector2d(57, 8)};

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
	private int updatesSinceCycleSwitch = 0;

	private int[] junctionCoords;

	private boolean doDrive = false;
	private boolean doElbow = false;
	private boolean doSlides = false;

	/**
	 * @brief set up required values for auto
	 *
	 * @param localizationType
	 */
	public void setup(Mode mode, int[] junctionCoords, SampleMecanumDrive.LocalizationType localizationType){
		initialize(hardwareMap, localizationType);

		this.currentMode = mode;

		this.camera = createCamera("Webcam 1");

		this.openCameraAndStreamAsync();

		this.signalSleeveDetector = new SignalSleeveDetector(this.camera);

		this.junctionCoords = junctionCoords;

		this.abe.arm.setSlidesRestingExtensionInches(20.0);
	}

	public void cycle(){
		switch(cycleState){
			case GRABBING: {
				// set states
				this.doDrive = true;
				this.doElbow = true;
				this.doSlides = true;

				this.abe.arm.setDesiredWristAngleDegrees(AbeConstants.WRIST_GRAB_ANGLE_DEGREES);

				if(this.updatesSinceCycleSwitch <= 1){
					this.abe.arm.setHandUnclamped();
				}

				// aim at stack
				this.aimAtConeStack();

				if(this.updatesSinceCycleSwitch > 2 && this.abe.isSteady() && !isEventScheduled(this.stateSwitchScheduler)){
					this.stateSwitchScheduler = getScheduledTime(GRABBING_TIME_SECONDS);
					this.abe.arm.setHandClamped();

					// update drive with imu while we're not moving
					correctPoseEstimateWithIMU();
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

				if(!isEventScheduled(this.stateSwitchScheduler)){
					this.stateSwitchScheduler = getScheduledTime(WRIST_LIFTING_TIME_SECONDS);
				}

				break;
			}

			case AIMING: {
				// aim to chosen junction
				this.aimToJunction(junctionCoords[0], junctionCoords[1], GlobalStorage.autoJunctionOffset);

				// set states
				this.doDrive = true;
				this.doElbow = true;
				this.doSlides = false;

				this.abe.arm.setHandClamped();
				this.abe.arm.setDesiredWristAngleDegrees(WRIST_LIFTING_ANGLE_DEGREES);

				// schedule switch when drive is close enough
				if(this.updatesSinceCycleSwitch > 2 && this.abe.drive.isSteady() && !isEventScheduled(this.stateSwitchScheduler)){
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

				this.abe.arm.setDesiredWristAngleDegrees(AbeConstants.WRIST_UP_ANGLE_DEGREES);
				this.abe.arm.setHandClamped();

				// schedule switch when slides are close enough
				if(this.updatesSinceCycleSwitch > 2 && this.abe.isSteady() && !isEventScheduled(this.stateSwitchScheduler)){
					this.stateSwitchScheduler = getScheduledTimeNow();
				}

				break;
			}

			case WRISTING: {
				// set states
				this.doDrive = true;
				this.doElbow = true;
				this.doSlides = true;

				this.abe.arm.setDesiredWristAngleDegrees(AbeConstants.WRIST_DROP_ANGLE_DEGREES);
				this.abe.arm.setHandClamped();

				if(gamepad2.a && !isEventScheduled(this.stateSwitchScheduler)){
					this.stateSwitchScheduler = getScheduledTime(WRISTING_TIME_SECONDS);
				}

				break;
			}

			case DROPPING: {
				// set states
				this.doDrive = true;
				this.doElbow = true;
				this.doSlides = true;

				this.abe.arm.setDesiredWristAngleDegrees(AbeConstants.WRIST_DROP_ANGLE_DEGREES);
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
				this.aimAtConeStack();

				this.abe.arm.setDesiredWristAngleDegrees(AbeConstants.WRIST_GRAB_ANGLE_DEGREES);
				this.abe.arm.setHandUnclamped();

				// switch when drive + elbow is steady
				if(this.updatesSinceCycleSwitch > 2 && this.abe.drive.isSteady() && this.abe.arm.isElbowSteady() && !isEventScheduled(this.stateSwitchScheduler)){
					this.stateSwitchScheduler = getScheduledTimeNow();
				}

				break;
			}
		}

		telemetry.addData("cycle state", this.cycleState.toString());

		if(isEventFiring(this.stateSwitchScheduler)){
			// increment cycle state
			this.cycleState = CycleState.increment(this.cycleState);

			// unschedule switch
			this.stateSwitchScheduler = UNSCHEDULED;

			// reset counter
			this.updatesSinceCycleSwitch = 0;
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
		return getPositionAccordingToMode(CONE_STACK_RIGHT_POSITION);
	}

	public void aimAtConeStack(){
		double height = this.getStackGrabHeight(getConesInStack());

		// determine stack position from mode
		Vector2d stackPosition = this.getConeStackPosition();

		// aim
		this.abe.aimAt(stackPosition.getX(), height, stackPosition.getY());
	}
}