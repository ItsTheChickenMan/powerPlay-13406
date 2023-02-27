package org.firstinspires.ftc.teamcode.lib.abe;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.lib.utils.GamepadEx;
import org.firstinspires.ftc.teamcode.lib.utils.GlobalStorage;
import org.firstinspires.ftc.teamcode.lib.utils.JunctionHelper;

/**
 * @brief abstract class for writing an op mode that utilizes abe
 *
 * abe bot is technically just a robot equipped with the ability to aim to a large volume of points in space around it.  this class takes abe bot and applies it specifically to a Power Play opmode
 *
 * this class works like LinearOpMode; the opmode class must inherit from this class and implement runOpMode
 */
@Config
public abstract class AbeOpMode extends LinearOpMode {
	// constants
	public static double GROUND_JUNCTION_HEIGHT_OFFSET_INCHES = 6.0;
	public static double LOW_JUNCTION_HEIGHT_OFFSET_INCHES = 0.0;
	public static double MEDIUM_JUNCTION_HEIGHT_OFFSET_INCHES = -4;
	public static double HIGH_JUNCTION_HEIGHT_OFFSET_INCHES = -4;

	// done this way to work with FTC dashboard (since it can't edit arrays yet?)
	public static double[] getJunctionHeightOffsetsInches(){
		return new double[]{GROUND_JUNCTION_HEIGHT_OFFSET_INCHES, LOW_JUNCTION_HEIGHT_OFFSET_INCHES, MEDIUM_JUNCTION_HEIGHT_OFFSET_INCHES, HIGH_JUNCTION_HEIGHT_OFFSET_INCHES}; // ground, low, medium, high
	}

	public static double BASE_CONE_STACK_HEIGHT = AbeTeleOp.ARM_GRAB_POSITION_INCHES.getY();
	public static double CONE_STACK_HEIGHT_INCREASE_RATE = 1.5;
	public static final Vector2d CONE_STACK_RIGHT_POSITION = new Vector2d(58.5, 7.6 - AbeConstants.WRIST_OFFSET_INCHES); // generally, this shouldn't have to depend on the wrist offset.  it does in this case because the code doesn't account for odd angles of the wrist; it was only ever meant to work at 0 degrees.  I could change it, but this is pretty much the only place where it actually matters.  It's just easier to hardcode it.

	// global timer
	private ElapsedTime globalTimer;

	// delta
	private double delta;
	private double lastTime; // time between updateDelta calls

	// ex gamepads
	protected GamepadEx gamepadEx1;
	protected GamepadEx gamepadEx2;

	/**
	 * @return the time in seconds since initialize() has last been called
	 */
	public double getGlobalTimeSeconds(){
		return globalTimer.seconds();
	}

	/**
	 * @return the amount of time passed since the last resetDelta call
	 */
	public double getDelta(){
		double time = this.getGlobalTimeSeconds();

		return time-lastTime;
	}

	/**
	 * @brief Reset the delta timer to 0 seconds
	 */
	public void resetDelta(){
		double time = this.getGlobalTimeSeconds();

		lastTime = time;
	}

	// event scheduling... //
	public static double UNSCHEDULED = 999999999;

	/**
	 * @param after the amount of time from the point at which the method is called that the event should fire
	 * @return the exact global time at which the event should fire, if it needs to fire some amount of seconds from now
	 */
	public double getScheduledTime(double after){
		return this.getGlobalTimeSeconds() + after;
	}

	/**
	 * @return the exact global time at which the event should fire, if it needs to fire right now
	 */
	public double getScheduledTimeNow(){
		return this.getScheduledTime(0.0);
	}

	/**
	 * @param scheduledTime
	 * @return true if an event is scheduled to occur currently.  still returns true if the scheduled time has passed (it's up to the programmer to reset it)
	 */
	public boolean isEventFiring(double scheduledTime){
		return scheduledTime < this.getGlobalTimeSeconds();
	}

	/**
	 * @param scheduledTime
	 * @return true if an event is scheduled to happen at some point in time.  not to be confused with isEventFiring
	 */
	public boolean isEventScheduled(double scheduledTime){
		return scheduledTime < UNSCHEDULED;
	}

	/**
	 * @brief Abe
	 */
	protected AbeBot abe;

	public void initialize(HardwareMap hardwareMap){
		this.initialize(hardwareMap, SampleMecanumDrive.LocalizationType.THREE_WHEEL);
	}

	public void setPoseEstimate(double x, double y, double r){
		this.abe.drive.setPoseEstimate(new Pose2d(x, y, r));
	}

	public void setPoseEstimate(Pose2d pose){
		this.abe.drive.setPoseEstimate(pose);
	}

	/**
	 * @brief initialize everything
	 *
	 * This should generally be called before calling any other method from AbeOpMode
	 *
	 * @param hardwareMap
	 */
	public void initialize(HardwareMap hardwareMap, SampleMecanumDrive.LocalizationType localizationType){
		// construct abe
		this.abe = new AbeBot(hardwareMap, localizationType);

		// construct global timer
		this.globalTimer = new ElapsedTime();

		// gamepads
		this.gamepadEx1 = new GamepadEx(gamepad1);
		this.gamepadEx2 = new GamepadEx(gamepad2);
	}

	/**
	 * @brief Aim to a junction using the junction coordinates provided
	 *
	 * Coordinates are measured from the bottom right corner of the field, starting at 0 (field perimeter) to 6 (field perimeter).
	 * Valid junctions are only at coordinates between 1 and 5.  Numbers outside of this range will make the method silently fail
	 *
	 * @param x x coordinate, from 1 to 5
	 * @param y y coordinate, from 1 to 5
	 */
	public void aimToJunction(int x, int y, Vector2d offset){
		// check coordinates
		if(!JunctionHelper.validateJunctionCoordinates(x, y)) return;

		// calculate junction coordinates
		Vector2d raw = JunctionHelper.snappedToRaw(x, y);

		raw = raw.plus(offset);

		// calculate junction height
		double height = JunctionHelper.getJunctionHeight(x, y);

		// modify height accordingly
		int index = JunctionHelper.getJunctionIndex(x, y);

		height += getJunctionHeightOffsetsInches()[index];

		// aim
		this.abe.aimAt(raw.getX(), height, raw.getY());
	}

	public void aimToJunction(int x, int y){
		this.aimToJunction(x, y, new Vector2d(0, 0));
	}

	/**
	 * @brief Aim to a junction using raw field coordinates.  This will snap the coordinates to the closest junction before aiming.
	 *
	 * @param raw raw field coordinates
	 */
	public void aimToJunctionRaw(Vector2d raw, Vector2d offset){
		int[] snapped = JunctionHelper.rawToSnapped(raw.getX(), raw.getY());

		this.aimToJunction(snapped[0], snapped[1], offset);
	}

	public void aimToJunctionRaw(Vector2d raw){
		this.aimToJunctionRaw(raw, new Vector2d(0, 0));
	}

	/**
	 * @param cones number of cones on the stack
	 * @return the physical height to grab the cone stack at
	 */
	public double getStackGrabHeight(int cones){
		return BASE_CONE_STACK_HEIGHT + CONE_STACK_HEIGHT_INCREASE_RATE*(cones-1);
	}

	public void updateControllerStates(){
		gamepadEx1.updateControllerStates();
		gamepadEx2.updateControllerStates();
	}

	/**
	 * @brief get the bot's current state and save appropriate values to GlobalStorage
	 */
	public void saveStateToGlobalStorage(){
		GlobalStorage.currentElbowAngleRadians = this.abe.arm.getElbowAngleRadians();
		GlobalStorage.currentSlidesExtension = this.abe.arm.getSlidesLengthInches();
		GlobalStorage.currentPose = this.abe.drive.getPoseEstimate();
	}

	/**
	 * @brief load the bot's current state from GlobalStorage.
	 *
	 * Does not reset GlobalStorage
	 */
	public void loadStateFromGlobalStorage(){
		// set elbow angle
		this.abe.arm.setElbowOffsetRadians(GlobalStorage.currentElbowAngleRadians);

		// set slides extension
		this.abe.arm.setSlidesExtensionOffsetInches(GlobalStorage.currentSlidesExtension - this.abe.arm.getSlidesBaseExtension());

		// set start point
		this.setPoseEstimate(GlobalStorage.currentPose);
	}

	/**
	 * @param pos
	 * @return the coordinates of pos if they were mirrored to the other side of the field
	 */
	public Vector2d getOppositeCoords(Vector2d pos){
		return new Vector2d(pos.getX(), JunctionHelper.FIELD_WIDTH - pos.getY());
	}

	public void correctPoseEstimateWithIMU(){
		Pose2d corrected = new Pose2d(
						this.abe.drive.getPoseEstimate().vec(),
						this.abe.drive.getRawExternalHeading()
		);

		setPoseEstimate(corrected);
	}
}
