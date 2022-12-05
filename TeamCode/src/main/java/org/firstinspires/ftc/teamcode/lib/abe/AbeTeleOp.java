package org.firstinspires.ftc.teamcode.lib.abe;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.firstinspires.ftc.teamcode.lib.utils.GlobalStorage;

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
	private AbeTeleOp.ControlMode controlMode = ControlMode.GRABBING;

	// "chosen" junction point
	private Vector2D chosenJunction = null;

	// are we extending, or no?
	private boolean extending = false;

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

	public void setup(){
		initializeAbe();

		// setup a specific mode
		if(this.controlMode == ControlMode.GRABBING){
			this.setupGrabbingMode();
		} else if(this.controlMode == ControlMode.AIMING){
			this.setupAimingMode();
		}
	}

	/**
	 * @brief set the current control mode
	 *
	 * @param mode
	 */
	public void setMode(AbeTeleOp.ControlMode mode){
		this.controlMode = mode;
	}

	public void checkJunctionAim(){
		// get aim vector
		double distance = 12.0;

		Vector2D desiredAimVector = new Vector2D(-gamepad2.left_stick_y, -gamepad2.left_stick_x);

		if(desiredAimVector.getNorm() < 0.75){
			return;
		}

		Vector2D aimVector = desiredAimVector.scalarMultiply(distance);

		Vector2D chooseVector = abe.drive.getPoseEstimateAsVector().add(aimVector);

		this.chosenJunction = chooseVector;
	}

	public void setupAimingMode(){
		// ensure clamped
		this.abe.arm.clampFingers();
	}

	public void setupGrabbingMode(){
		// ensure unclamped
		this.abe.arm.unclampFingers();

		// move the arm to a good grabbing position
		// FIXME: add to AbeConstants
		//this.abe.arm.aimAt(20, 4.5 - AbeConstants.ARM_VERTICAL_OFFSET_INCHES); // relative to arm position, not bot position...
	}

	public void cleanupAimingMode(){
		// clear the aim point
		this.abe.clearPoint();

		// tell drive to lock current orientation
		this.abe.drive.aimAtCurrentAngle();
	}

	public void cleanupGrabbingMode(){
		// just here if I need it...
	}

	public void updateControllerStates(){
		this.g2ALastPressed = gamepad2.a;
		this.g2BLastPressed = gamepad2.b;
	}

	public void update(){
		// update delta
		this.updateDelta();

		double delta = this.getDelta();

		// different process depending on mode
		switch(this.controlMode){

			case AIMING: {
				// determine current aim state (which part(s) of the arm are active?)
				this.extending = gamepad2.left_trigger > 0.05; // TODO: adjustable slide length based on strength of trigger?

				// check junction being aimed at
				if(!this.extending) {
					this.checkJunctionAim();
				}

				// check for correction
				// TODO: add correction rate to constants
				double correctionRate = 3; // in inches / second

				Vector2D poseCorrection = new Vector2D(gamepad2.right_stick_y*delta*correctionRate, gamepad2.right_stick_x*delta*correctionRate);

				if(poseCorrection.getNorm() > 0.05 && this.extending){
					this.abe.drive.addToOffset(poseCorrection.getX(), poseCorrection.getY());
				}

				//GlobalStorage.globalTelemetry.addData("x", this.chosenJunction.getX());
				//GlobalStorage.globalTelemetry.addData("y", this.chosenJunction.getY());

				if(this.chosenJunction != null) {
					this.aimAtJunctionRaw(this.chosenJunction.getX(), this.chosenJunction.getY(), true, true, this.extending && !isEventScheduled(this.switchModeSchedule));
				}

				// check for deposit request
				if(gamepad2.a){
					// deposit
					this.abe.arm.unclampFingers();

					// schedule mode switch
					this.switchModeSchedule = getScheduledTime(1.0);
				}

				// check if mode switch has been requested
				if( isScheduledEventHappening(this.switchModeSchedule) ){
					// unschedule the switch
					this.switchModeSchedule = UNSCHEDULED;

					// cleanup + setup
					this.cleanupAimingMode();
					this.setupGrabbingMode();

					// switch modes
					this.controlMode = ControlMode.GRABBING;
				}

				break;
			}

			case GRABBING: {
				// do drive train aim
				// could probably just use atan2 here?  not urgent but it's something to note
				Vector2d aimVector = new Vector2d(-gamepad1.right_stick_y, -gamepad1.right_stick_x);
				double heading = aimVector.angle();

				// move drive to selected angle, otherwise maintain current angle
				if(aimVector.norm() > 0.5) {
					this.abe.drive.aimAtAngleRadians(heading);
				}

				if(gamepad2.left_trigger > 0.05){
					this.abe.arm.aimAt(20, 4.5 - AbeConstants.ARM_VERTICAL_OFFSET_INCHES); // relative to arm position, not bot position...
				} else {
					this.abe.arm.aimAt(6, 20);
				}

				// check for grab
				if(gamepad2.a){
					this.abe.arm.clampFingers();

					// schedule mode switch in half a second
					this.switchModeSchedule = getScheduledTime(0.5);
				}

				// check if mode switch has been requested
				if( isScheduledEventHappening(this.switchModeSchedule) ){
					// unschedule the switch
					this.switchModeSchedule = UNSCHEDULED;

					// cleanup + setup
					this.cleanupGrabbingMode();
					this.setupAimingMode();

					// switch modes
					this.controlMode = ControlMode.AIMING;
				}
			}
		}

		this.abe.update();

		this.updateControllerStates();
	}
}
