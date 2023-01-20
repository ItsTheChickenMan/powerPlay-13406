package org.firstinspires.ftc.teamcode.lib.abe;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.firstinspires.ftc.teamcode.lib.utils.AngleHelper;
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

	private double lockPoint = 0;
	private boolean lockPointSet = false;

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
		this.chosenJunction = null;
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
		//this.abe.drive.aimAtCurrentAngle();
	}

	public void cleanupGrabbingMode(){
		// just here if I need it...
	}

	public void updateControllerStates(){
		this.g2ALastPressed = gamepad2.a;
		this.g2BLastPressed = gamepad2.b;
	}

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

		// mode processing
		// different process depending on mode
		switch(this.controlMode){
			// aiming at junctions
			case AIMING: {

				// determine current aim state (which part(s) of the arm are active?)
				this.extending = gamepad2.left_trigger > 0.05; // TODO: adjustable slide length based on strength of trigger?

				// check junction being aimed at
				if(!this.extending) {
					this.checkJunctionAim();
				}

				// check for correction
				// TODO: add correction rate to constants
				double correctionRate = 5; // in inches / second

				Vector2D poseCorrection = new Vector2D(gamepad2.right_stick_y*delta*correctionRate, gamepad2.right_stick_x*delta*correctionRate);

				if(poseCorrection.getNorm() > 0.05 && this.extending && this.chosenJunction != null){
					this.abe.drive.addToOffset(poseCorrection.getX(), poseCorrection.getY());
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

					// TODO: programmable constant
					if(distance > 4.0) {
						this.aimAtJunctionRaw(this.chosenJunction.getX(), this.chosenJunction.getY(), true, true, this.extending || isEventScheduled(this.switchModeSchedule));

						// keep wrist at 0 degrees
						this.abe.arm.positionWristDegrees(0.0);
					}
				} else {
					// ensure that we're not aiming at anything
					this.abe.clearPoint();

					// aim at a safe spot for maneuverability
					this.abe.arm.aimAt(6, 20);

					// do regular drive rotation
					//this.checkDriveRotation();

					// update wrist angle slightly as a visual indication of lack of chosen junction
					this.abe.arm.positionWristDegrees(35.0);
				}

				// check for deposit request
				if(gamepad2.a){
					// deposit
					this.abe.arm.unclampFingers();

					// schedule mode switch
					this.switchModeSchedule = getScheduledTime(0.25);
				}

				if(gamepad2.x){
					this.chosenJunction = null;
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
				// do drive rotation calculation, accounting for slowmode setting
				this.checkDriveRotation(Math.min(1.2 - speedTrigger, 1.0));

				boolean down = gamepad2.right_trigger > 0.05;

				if(down){
					// 21 7/8, as negotiated by Saeid and Phoenix
					this.abe.arm.aimAt(21.875, 3 - AbeConstants.ARM_VERTICAL_OFFSET_INCHES); // relative to arm position, not bot position...

					if(gamepad2.dpad_down){
						this.abe.arm.addToElbowOffsetDegrees(2 * delta);
					} else if (gamepad2.dpad_up){
						this.abe.arm.addToElbowOffsetDegrees(-2 * delta);
					}
				} else {
					this.abe.arm.aimAt(6, 20);
				}

				// check for grab
				if(gamepad2.a){
					this.abe.arm.clampFingers();

					// schedule mode switch in half a second
					this.switchModeSchedule = getScheduledTime(0.4);
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
