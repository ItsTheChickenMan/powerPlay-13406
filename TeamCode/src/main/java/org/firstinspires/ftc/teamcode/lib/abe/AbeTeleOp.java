package org.firstinspires.ftc.teamcode.lib.abe;

import android.provider.Settings;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.firstinspires.ftc.teamcode.lib.utils.AngleHelper;
import org.firstinspires.ftc.teamcode.lib.utils.GlobalStorage;
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
	private AbeTeleOp.ControlMode controlMode = ControlMode.GRABBING;

	// "chosen" junction point
	private Vector2D chosenJunction = null;

	private double lockPoint = 0;
	private boolean lockPointSet = false;

	private Vector2D substationOffsetLeft = new Vector2D(0, 0);
	private Vector2D substationOffsetRight = new Vector2D(0, 0);

	// SPECIFICALLY CONTROLLER STATES //
	private boolean g2ALastPressed = false;
	private boolean g2BLastPressed = false;
	private boolean g2DpadUpLastPressed = false;
	private boolean g2DpadDownLastPressed = false;

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

		this.abe.drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

	public void checkDriveRotation(double speed){
		// aim vector for drive rotation
		Vector2D aimVector = new Vector2D(-gamepad1.right_stick_y, -gamepad1.right_stick_x);

		if(aimVector.getNorm() < 0.25) return;

		aimVector = aimVector.normalize();

		// get rotational power
		double rotationSpeed = 24.0 * speed;

		Vector2D forward = this.abe.drive.getForwardVector();

		double dot = forward.dotProduct(aimVector);

		double rot = (1.0-dot) * rotationSpeed;

		double forwardVectorRotation = Math.atan2(forward.getY(), forward.getX());
		double aimVectorRotation = Math.atan2(aimVector.getY(), aimVector.getX());

		//GlobalStorage.globalTelemetry.addData("forward vector rotation", Math.toDegrees(forwardVectorRotation));
		//GlobalStorage.globalTelemetry.addData("aim vector rotation", Math.toDegrees(aimVectorRotation));

		// get direction
		double direction = AngleHelper.angularDistanceRadians(aimVectorRotation, forwardVectorRotation);
		direction = Math.signum(direction);

		this.abe.drive.rotate(rot * direction);

		/*
		double heading = AbeDrive.apacheVectorToRRVector(aimVector).angle();

		// aim at where the stick is pointing, otherwise maintain current heading
		// this effectively creates a "driver centric rotation" where the direction of the joystick more or less dictates the direction that the front will travel in rotation
		if(aimVector.getNorm() > 0.5) {
			this.lockPointSet = false;

			this.abe.drive.aimAtAngleRadians(heading, speed);
		} else {
			if(!this.lockPointSet){
				this.lockPoint = this.abe.drive.getPoseEstimate().getHeading();

				this.lockPointSet = true;
			}

			this.abe.drive.aimAtAngleRadians(this.lockPoint, speed);
		}
		*/
	}

	public void checkJunctionAim(){
		// get aim vector
		double distance = 16.0;

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

		// deselect selected junction
		// NOTE: commenting this out because it slightly slows down tall cycles a bit because I have to re-aim the robot at the junction each time.
		// this way, it aims itself back at the previously selected junctions as a convenience (which is the near tall in most cases)

		// check if we're within distance
		/*if(this.chosenJunction != null) {
			double distanceSq = this.abe.drive.getPoseEstimateAsVector().subtract(this.chosenJunction).getNormSq();
			double maxDistance = 16;

			if (distanceSq > maxDistance * maxDistance) {
				// too far, clear
				this.chosenJunction = null;
			}
		}*/

		// NOTE: I added this line back, despite the comment above, because losing control over when the robot aimed proved to be too difficult to control to be worth the additional speed.  the speed boost was fairly minimal anyways.
		this.chosenJunction = null;
	}

	public void setupGrabbingMode(){
		// ensure unclamped
		this.abe.arm.unclampFingers();

		// enable aim to substation by default
		this.aimingToSubstation = true;

		// move the arm to a good grabbing position
		//this.abe.arm.aimAt(20, 4.5 - AbeConstants.ARM_VERTICAL_OFFSET_INCHES); // relative to arm position, not bot position...
	}

	public void cleanupAimingMode(){
		// clear the aim point
		this.abe.clearPoint();

		// tell drive to lock current orientation
		//this.abe.drive.aimAtCurrentAngle();
	}

	public void cleanupGrabbingMode(){
		// just here if I need it...
	}

	// FIXME: controller states don't work?
	public void updateControllerStates(){
		this.g2ALastPressed = gamepad2.a;
		this.g2BLastPressed = gamepad2.b;
		this.g2DpadUpLastPressed = gamepad2.dpad_up;
		this.g2DpadDownLastPressed = gamepad2.dpad_down;
	}

	// deposit switch state scheduler
	// for switching from wrist down to drop
	private double depositStateSchedule = UNSCHEDULED;

	// selected stack height
	private int selectedStackHeight = 4;

	// whether or not the arm is aiming at the stack in grab mode
	private boolean doingStack = false;

	private boolean stackHeightChangedLastFrame = false;

	// if we're currently using auto-aim to substation if within distance
	private boolean aimingToSubstation = true;
	private boolean substationChangedLastFrame = false;

	public void update(){
		// update/fetch delta
		this.updateDelta();

		double delta = this.getDelta();

		// drive
		double forward = -gamepad1.left_stick_y;
		double strafe = gamepad1.left_stick_x;
		//double speed = 30.0 * (1.20 - Math.max(gamepad1.left_trigger, gamepad1.right_trigger));

		// determine speed (we use both triggers because it's hard to remember which is which sometimes)
		double speedTrigger = Math.max(gamepad1.left_trigger, gamepad1.right_trigger);
		double speed = 30 * (1.35 - speedTrigger);

		// drive field oriented
		this.abe.drive.driveFieldOriented(forward*speed, strafe*speed);

		// substation pickup spot
		Vector2D pickupSpot = this.getPickupSpot();

		// determine current aim state (which part(s) of the arm are active?)
		boolean extending = gamepad2.left_trigger > 0.05; // TODO: adjustable slide length based on strength of trigger?
		boolean onLeftSide = this.abe.drive.getPoseEstimate().getY() > JunctionHelper.FIELD_LENGTH/2;

		if(extending){
			// check for correction
			// TODO: add correction rate to constants
			double correctionRate = 6.0 * (0.5 + gamepad2.right_trigger); // in inches / second

			Vector2D poseCorrection = new Vector2D(gamepad2.right_stick_y*delta*correctionRate, gamepad2.right_stick_x*delta*correctionRate);

			if(poseCorrection.getNorm() > 0.05 && (this.chosenJunction != null || this.aimingToSubstation)) {
				this.abe.drive.addToOffset(poseCorrection.getX(), poseCorrection.getY());
			}
		}

		// mode processing
		// different process depending on mode
		switch(this.controlMode){
			// aiming at junctions
			case AIMING: {
				// check junction being aimed at
				if(!extending) {
					this.checkJunctionAim();
				}

				//GlobalStorage.globalTelemetry.addData("x", this.chosenJunction.getX());
				//GlobalStorage.globalTelemetry.addData("y", this.chosenJunction.getY());

				// log state
				/*if(this.chosenJunction != null){
					GlobalStorage.globalTelemetry.addData("chosen junction", this.chosenJunction.getX() + ", " + this.chosenJunction.getY());
				} else {
					GlobalStorage.globalTelemetry.addData("chosen junction", "null");
				}*/

				if(this.chosenJunction != null) {
					double distance = Vector2D.distance(this.chosenJunction, this.abe.drive.getPoseEstimateAsVector());

					GlobalStorage.globalTelemetry.addData("chosen junction", this.chosenJunction.getX() + ", " + this.chosenJunction.getY());

					// TODO: programmable constant
					if(distance > 4.0) {
						this.aimAtJunctionRaw(this.chosenJunction.getX(), this.chosenJunction.getY(), true, true, extending && this.abe.drive.getAimErrorDegrees() < 35  /* || isEventScheduled(this.switchModeSchedule)*/);

						// keep wrist at 0 degrees
						this.abe.arm.addToWristAngleDegrees(0.0);
					}
				} else {
					// ensure that we're not aiming at anything
					this.abe.clearPoint();

					// aim at a safe spot for maneuverability
					this.abe.arm.aimAt(6, 20);

					// do regular drive rotation
					// TODO: why was this commented out?  thanks for being a dickhead and not leaving a comment, phoenix
					// 	-phoenix
					this.checkDriveRotation(Math.min(1.2 - speedTrigger, 1.0));

					// update wrist angle slightly as a visual indication of lack of chosen junction
					// TODO: this doesn't work?
					this.abe.arm.addToWristAngleDegrees(55);
				}

				// check for selection cancel
				if(gamepad2.x){
					this.chosenJunction = null;
				}

				boolean wristIsDown = false;

				// check for wrist preview (also updates wrist for deposit state)
				if(gamepad2.right_bumper || isEventScheduled(this.depositStateSchedule)){
					this.abe.arm.addToWristAngleDegrees(AbeConstants.WRIST_DEPOSITING_ANGLE_DEGREES);
					wristIsDown = true;
				} else {
					this.abe.arm.addToWristAngleDegrees(AbeConstants.WRIST_HOLDING_ANGLE_DEGREES);
				}

				// check for deposit request
				if(gamepad2.a && !isEventScheduled(this.depositStateSchedule)){
					double time = wristIsDown ? 0.0 : 0.25;

					// schedule mode switch
					this.depositStateSchedule = getScheduledTime(time);
				}

				// check if deposit state is scheduled to change
				if( isScheduledEventHappening(this.depositStateSchedule) ){
					// drop cone
					this.abe.arm.unclampFingers();

					// unschedule deposit state
					this.depositStateSchedule = UNSCHEDULED;

					// schedule mode switch
					this.switchModeSchedule = getScheduledTime(0.1);
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

				//GlobalStorage.globalTelemetry.addData("extending", this.extending);

				break;
			}

			case GRABBING: {
				boolean down = gamepad2.right_trigger > 0.05;
				boolean modeSwitch = gamepad2.b;

				if(down){
					// clear point
					this.abe.clearPoint();

					// do drive rotation calculation, accounting for slowmode setting
					this.checkDriveRotation(Math.min(1.2 - speedTrigger, 1.0));

					double height;

					// doing stack aim?
					if (this.doingStack) {
						// snap stack height
						this.selectedStackHeight = Math.max(Math.min(this.selectedStackHeight, 4), 0);

						// get height
						height = this.getConeStackHeight(this.selectedStackHeight);

						// check for stack change
						if (gamepad2.dpad_up && !this.stackHeightChangedLastFrame) {
							this.selectedStackHeight++;
						}

						if (gamepad2.dpad_down && !this.stackHeightChangedLastFrame) {
							this.selectedStackHeight--;
						}

						if (gamepad2.dpad_down || gamepad2.dpad_up) {
							this.stackHeightChangedLastFrame = true;
						} else {
							this.stackHeightChangedLastFrame = false;
						}
					}
					// otherwise go to normal height
					else {
						height = AbeConstants.DEFAULT_GRABBING_HEIGHT;
					}

					// check if stack aim is pressed
					if (gamepad2.dpad_up && !this.doingStack) {
						// enable stack aim height
						this.doingStack = true;
					}

					height -= AbeConstants.ARM_VERTICAL_OFFSET_INCHES;

					// 21 7/8, as negotiated by Saeid and Phoenix
					this.abe.arm.aimAt(21.875, height);
				} else {
					// clear point
					this.abe.clearPoint();

					// disable stack
					this.doingStack = false;

					// do drive rotation calculation, accounting for slowmode setting
					this.checkDriveRotation(Math.min(1.2 - speedTrigger, 1.0));

					this.abe.arm.aimAt(6, 20);
				}

				// check for grab
				if(gamepad2.a && !isEventScheduled(this.switchModeSchedule)){
					this.abe.arm.clampFingers();

					// schedule mode switch in half a second
					this.switchModeSchedule = getScheduledTime(0.4);
				}

				// check if mode switch has been requested
				if( isScheduledEventHappening(this.switchModeSchedule) ){
					// unschedule the switch
					this.switchModeSchedule = UNSCHEDULED;

					// decrease stack size for grab
					if(this.doingStack){
						this.selectedStackHeight--;
					}

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

	public Vector2D getPickupSpot(){
		int side = this.abe.drive.getPoseEstimate().getY() > JunctionHelper.FIELD_LENGTH/2 ? 1 : -1;

		Vector2D pickupSpot = AbeConstants.SUBSTATION_DROP_SPOT_CENTER.add(new Vector2D(0, AbeConstants.SUBSTATION_DROP_SPOT_OFFSET*side));

		return pickupSpot;
	}
}
