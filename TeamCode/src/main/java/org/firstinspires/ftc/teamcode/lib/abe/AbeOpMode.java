package org.firstinspires.ftc.teamcode.lib.abe;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.firstinspires.ftc.teamcode.lib.utils.AprilTagDetector;
import org.firstinspires.ftc.teamcode.lib.utils.GlobalStorage;
import org.firstinspires.ftc.teamcode.lib.utils.JunctionHelper;

/**
 * @brief Abstract class containing required values and methods for using AbeBot in a general OpMode
 *
 * More specific methods for autonomous and teleop should be contained in AbeAutonomous and AbeTeleOp, respectively
 */
public abstract class AbeOpMode extends LinearOpMode {
	protected static final double BASE_CONE_STACK_HEIGHT = 3.0;
	protected static final double CONE_STACK_HEIGHT_INCREASE_RATE = 1.375;

	public double getConeStackHeight(int cone){
		// calculate height
		double height = AbeOpMode.BASE_CONE_STACK_HEIGHT + AbeOpMode.CONE_STACK_HEIGHT_INCREASE_RATE*cone;

		return height;
	}

	// abe bot
	protected AbeBot abe;

	// event scheduling
	protected ElapsedTime timer = new ElapsedTime();
	public static final double UNSCHEDULED = 999999999999999.; // this is hacky, but I love it

	// delta calc
	private double lastTime;
	private double delta;

	public double getDelta(){
		return delta;
	}

	public void updateDelta(){
		double curTime = timer.seconds();
		this.delta = curTime - this.lastTime;
		this.lastTime = curTime;
	}

	/**
	 * @brief Returns the current time + after, used for scheduling timed events
	 *
	 * @param after time until event, in seconds
	 * @return time at which the event will occur
	 */
	public double getScheduledTime(double after){
		return timer.seconds() + after;
	}

	/**
	 * @brief Tells whether a scheduled event (scheduled by getScheduledTime) is scheduled to happen now
	 *
	 * @param scheduledTime time at which event was scheduled
	 * @return true if it should happen now, false otherwise
	 */
	public boolean isScheduledEventHappening(double scheduledTime){
		return timer.seconds() > scheduledTime;
	}

	/**
	 * @brief returns true if an event is currently scheduled to occur at some point, otherwise false
	 *
	 * @param scheduledTime
	 * @return true if scheduled, false if not
	 */
	public boolean isEventScheduled(double scheduledTime){
		return scheduledTime != UNSCHEDULED;
	}

	/**
	 * @brief Log an error to telemetry and freeze the program until stopped
	 */
	public void logError(String errorMessage){
		telemetry.addData("error", errorMessage);
		telemetry.update();

		while(!isStopRequested());
	}

	/**
	 * @brief set the starting point of the robot, in roadrunner coordinates
	 *
	 * Normally, the bot's coordinates begin at 0, 0 with a rotation of 0.  this allows you to set an offset for them
	 *
	 * @param x x offset, in roadrunner coords
	 * @param y y offset, in roadrunner coords
	 * @param r rotation, in radians
	 */
	public void setStartPoint(double x, double y, double r){
		this.abe.setPoseEstimate(x, y, r);
	}

	public void setStartPoint(Pose2d pose){
		this.abe.setPoseEstimate(pose);
	}

	public void aimAtJunctionRaw(double x, double y, boolean doDrive, boolean doElbow, boolean doSlides, Vector2D offset){
		// get height
		double height = JunctionHelper.getJunctionHeightFromRaw(x, y);

		// ensure that the height is valid
		if(!JunctionHelper.isValidHeight(height)) return;

		// add the height offset
		if(height == JunctionHelper.GROUND_JUNCTION_HEIGHT_INCHES) {
			// special offset for ground
			// FIXME: constant?
			height += 5.0;
		}	else if(height == JunctionHelper.LOW_JUNCTION_HEIGHT_INCHES) {
			height += 2.5;
		} else {
			height += AbeConstants.ARM_POLE_HEIGHT_OFFSET_INCHES;
		}

		// get real x and y
		double[] coords = JunctionHelper.snappedToRaw(JunctionHelper.rawToSnapped(x, y)); // TODO: I hate this
		double rx = coords[0] + offset.getX();
		double ry = coords[1] + offset.getY();

		GlobalStorage.globalTelemetry.addData("rx", rx);
		GlobalStorage.globalTelemetry.addData("ry", ry);

		// aim
		this.abe.aimAt(rx, height, ry, doDrive, doElbow, doSlides);
	}

	public void aimAtJunctionRaw(double x, double y, boolean doDrive, boolean doElbow, boolean doSlides) {
		this.aimAtJunctionRaw(x, y, doDrive, doElbow, doSlides, new Vector2D(0, 0));
	}

	public void aimAtJunctionRaw(double x, double y){
		this.aimAtJunctionRaw(x, y, true, true, true);
	}

	/**
	 * @brief Initialize abe bot
	 */
	public void initializeAbe(){
		try {
			this.abe = new AbeBot(hardwareMap);
		} catch (Exception e) {
			logError(e.getMessage());
		}
	}

	/**
	 * @brief get the bot's current state and save appropriate values to GlobalStorage
	 */
	public void saveStateToGlobalStorage(){
		GlobalStorage.currentElbowAngleRadians = this.abe.arm.getElbowAngleRadians();
		GlobalStorage.currentSlidesExtension = this.abe.arm.getSlidesExtension();
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
		this.abe.arm.setSlidesExtensionOffset(GlobalStorage.currentSlidesExtension - this.abe.arm.getSlidesBaseExtension());

		// set start point
		this.setStartPoint(GlobalStorage.currentPose);
	}
}