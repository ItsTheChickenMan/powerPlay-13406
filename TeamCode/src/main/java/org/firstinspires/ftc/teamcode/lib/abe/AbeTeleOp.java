package org.firstinspires.ftc.teamcode.lib.abe;

import org.apache.commons.math3.geometry.Vector;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.firstinspires.ftc.teamcode.lib.utils.JunctionHelper;

/**
 * @brief Abstract class containing required values and methods for using AbeBot during the Teleop period
 *
 * This should only be for methods specifically tailored for TeleOp.  Methods that are tailored for Autonomous should be in AbeAutonomous, and methods for both should be in AbeOpMode
 *
 * Inherit from this class instead of LinearOpMode to use the methods
 */
public abstract class AbeTeleOp extends AbeOpMode {
	// SCHEDULED EVENT TIMES //
	private double switchModeSchedule = UNSCHEDULED;

	// STATES n STUFF //

	// what mode are we in?
	private AbeTeleOp.ControlMode mode = ControlMode.GRABBING;

	// "chosen" junction point
	private Vector2D chosenJunction;

	// are we locked onto a junction currently?
	private boolean lockedOnJunction = false;

	// SPECIFICALLY CONTROLLER STATES //
	private boolean g2ALastPressed = false;
	private boolean g2BLastPressed = false;

	// ENUMS //

	/**
	 * @brief enum for the control mode
	 */
	public static enum ControlMode {
		GRABBING,
		AIMING
	}

	// METHODS //

	/**
	 * @brief set the current control mode
	 *
	 * @param mode
	 */
	public void setMode(AbeTeleOp.ControlMode mode){
		this.mode = mode;
	}

	public void checkJunctionAim(){
		// get aim vector
		double distance = 12.0;

		Vector2D desiredAimVector = new Vector2D(gamepad2.left_stick_x, -gamepad2.left_stick_y);

		Vector2D aimVector = desiredAimVector.scalarMultiply(distance);

		Vector2D chooseVector = abe.drive.getPoseEstimateAsRegularVector().add(aimVector);

		this.chosenJunction = chooseVector;
	}

	public void cleanupAimMode(){
		// clear the aim point
		this.abe.clearPoint();

		// unlock
		this.lockedOnJunction = false;

		// move the arm to a good grabbing position
		// FIXME: add to AbeConstants
		this.abe.arm.aimAt(20, 6.5);
	}

	public void updateControllerStates(){
		this.g2ALastPressed = gamepad2.a;
		this.g2BLastPressed = gamepad2.b;
	}

	public void update(){
		// different process depending on mode
		switch(this.mode){

			case AIMING: {
				// check junction being aimed at
				if(!this.lockedOnJunction) {
					this.checkJunctionAim();
				}

				// check if we should lock onto the current junction
				if(gamepad2.b && !this.g2BLastPressed){
					this.lockedOnJunction = !this.lockedOnJunction;
				}

				// determine current aim state (which part(s) of the arm are active?)
				boolean extendSlides = gamepad2.left_trigger > 0.05; // TODO: adjustable slide length based on strength of trigger?

				aimAtJunctionRaw(this.chosenJunction.getX(), this.chosenJunction.getY(), true, true, extendSlides);

				// check for deposit request
				if(gamepad2.a){
					// deposit
					this.abe.arm.unclampFingers();

					// schedule mode switch
					this.switchModeSchedule = getScheduledTime(0.5);
				}

				// check if mode switch has been requested
				if( isScheduledEventHappening(this.switchModeSchedule) ){
					// unschedule the switch
					this.switchModeSchedule = UNSCHEDULED;

					// do a bit of cleanup before we switch
					this.cleanupAimMode();

					// switch modes
					this.mode = ControlMode.GRABBING;
				}

				break;
			}

			case GRABBING: {
				// do drive train aim
				Vector2D aimVector = new Vector2D(-gamepad1.right_stick_y, gamepad1.right_stick_x);

				// check for grab
				if(gamepad2.a){
					// clamp fingies
					this.abe.arm.clampFingers();

					// schedule mode switch in half a second
					this.switchModeSchedule = getScheduledTime(0.5);
				}

				// check if mode switch has been requested
				if( isScheduledEventHappening(this.switchModeSchedule) ){
					// unschedule the switch
					this.switchModeSchedule = UNSCHEDULED;

					// switch modes
					this.mode = ControlMode.AIMING;
				}
			}
		}

		this.abe.update();

		this.updateControllerStates();
	}
}
