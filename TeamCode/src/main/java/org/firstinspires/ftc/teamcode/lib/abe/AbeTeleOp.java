package org.firstinspires.ftc.teamcode.lib.abe;

import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public abstract class AbeTeleOp extends AbeOpMode {
	// constants
	public static double GRAB_MODE_PICKUP_TIME_SECONDS = 0.25;
	public static double AIM_MODE_DEPOSIT_TIME_SECONDS = 0.1;

	public static Vector2d ARM_DEFAULT_POSITION_INCHES = new Vector2d(15, 8);
	public static Vector2d ARM_GRAB_POSITION_INCHES = new Vector2d(19, 2);

	public static double BASE_POSE_CORRECTION_RATE_INCHES_PER_SECOND = 6.0;
	public static double POSE_CORRECTION_RATE_MULTIPLIER = 2.5;

	public static double JUNCTION_AIM_DISTANCE_INCHES = 16.0;

	// modes
	public enum ControlMode {
		GRABBING,
		AIMING
	}

	public enum GrabMode {
		GROUND,
		STACK
	}

	// states //
	private ControlMode currentControlMode = ControlMode.GRABBING;
	private GrabMode currentGrabMode = GrabMode.GROUND;
	private int aimStackHeight = 5;

	private Vector2d chosenJunction;

	// schedules
	private double switchModeSchedule = UNSCHEDULED;

	// methods
	public void setup(SampleMecanumDrive.LocalizationType localizationType){
		initialize(hardwareMap, localizationType);

		if(currentControlMode == ControlMode.GRABBING){
			this.setupGrabMode();
		} else if(currentControlMode == ControlMode.AIMING){
			this.setupAimMode();
		}
	}

	public void setupAimMode(){
		// reset chosen junction
		this.chosenJunction = null;
	}

	public void setupGrabMode(){
		// default to ground mode
		this.currentGrabMode = GrabMode.GROUND;

		// clear aim point
		this.abe.clearAim();

		// unclamp
		this.abe.arm.setHandUnclamped();
	}

	public void update(){
		// get delta
		double delta = getDelta();

		resetDelta();

		// update controller states
		updateControllerStates();

		// load drive controller states
		double forward = -gamepad1.left_stick_y;
		double strafe = gamepad1.left_stick_x;
		double rotate = gamepad1.right_stick_x;

		// drive
		this.abe.drive.driveFieldOriented(forward, strafe, rotate);

		// control logic
		switch(this.currentControlMode){
			// mode for grabbing cones, doesn't really use auto aim
			case GRABBING: {
				// load controller states
				boolean down = gamepad2.right_trigger > 0.05;
				boolean stackUp = gamepadEx2.dpad_up_pressed;
				boolean stackDown = gamepadEx2.dpad_down_pressed;
				boolean switchMode = gamepadEx2.a_pressed;

				telemetry.addData("down", down);
				telemetry.addData("stackUp", stackUp);
				telemetry.addData("stackDown", stackDown);
				telemetry.addData("switchMode", switchMode);

				if(!down) {
					// default to ground mode
					this.currentGrabMode = GrabMode.GROUND;

					// move to default position
					this.abe.arm.aimAt(ARM_DEFAULT_POSITION_INCHES.getX(), ARM_DEFAULT_POSITION_INCHES.getY());
				} else {
					// check grab mode
					switch (this.currentGrabMode) {
						case GROUND: {
							// go to down position
							this.abe.arm.aimAt(ARM_GRAB_POSITION_INCHES.getX(), ARM_GRAB_POSITION_INCHES.getY());

							// switch condition
							if(stackUp){
								// switch mode
								this.currentGrabMode = GrabMode.STACK;
							}

							break;
						}

						case STACK: {
							// get stack height
							double grabHeight = this.getStackGrabHeight(this.aimStackHeight);

							// aim
							this.abe.arm.aimAt(ARM_GRAB_POSITION_INCHES.getX(), grabHeight);

							// adjust stack height
							if(stackDown){
								this.aimStackHeight--;
							} else if(stackUp){
								this.aimStackHeight++;
							}

							// snap stack
							this.aimStackHeight = Math.min(5, this.aimStackHeight);
							this.aimStackHeight = Math.max(1, this.aimStackHeight);

							break;
						}
					}
				}

				// switch condition
				if(switchMode && !isEventScheduled(this.switchModeSchedule)){
					// schedule switch
					this.switchModeSchedule = getScheduledTime(GRAB_MODE_PICKUP_TIME_SECONDS);
				}

				// switch
				if(isEventFiring(this.switchModeSchedule)){
					// switch mode
					this.currentControlMode = ControlMode.AIMING;

					// unschedule event
					this.switchModeSchedule = UNSCHEDULED;

					// setup aim
					this.setupAimMode();

					// clamp
					this.abe.arm.setHandClamped();
				}

				this.abe.update();

				break;
			}

			// mode for depositing, heavily relies on auto aim
			case AIMING: {
				// get controller states
				boolean slides = gamepad2.left_trigger > 0.05;
				boolean wristing = gamepad2.right_bumper;
				boolean drop = gamepad2.a;

				double junctionSelectionX = -gamepad2.left_stick_x;
				double junctionSelectionY = -gamepad2.left_stick_y;

				double poseCorrectionX = gamepad2.right_stick_x;
				double poseCorrectionY = gamepad2.right_stick_y;
				double correctionSpeed = gamepad2.right_trigger;

				// check chosen junction
				if(hasChosenJunction()) {
					// check slides
					if(slides){
						// check pose correction
						Vector2d poseCorrection = new Vector2d(poseCorrectionX, poseCorrectionY);

						// correction rate
						double b = BASE_POSE_CORRECTION_RATE_INCHES_PER_SECOND;
						double m = POSE_CORRECTION_RATE_MULTIPLIER;
						double correctionRate = b * (1 + correctionSpeed * (m - 1));

						correctionRate *= delta;

						poseCorrection = poseCorrection.times(correctionRate);

						this.abe.drive.addToPoseOffset(poseCorrection);
					}

					// check wrist
					if(wristing){
						this.abe.arm.setDesiredWristAngleDegrees(AbeConstants.WRIST_DROP_ANGLE_DEGREES);
					} else {
						this.abe.arm.setDesiredWristAngleDegrees(AbeConstants.WRIST_UP_ANGLE_DEGREES);
					}

					// aim
					aimToJunctionRaw(this.chosenJunction);
				} else {
					// clear aim
					this.abe.clearAim();

					// aim at default
					this.abe.arm.aimAt(ARM_DEFAULT_POSITION_INCHES.getX(), ARM_DEFAULT_POSITION_INCHES.getY());

					this.abe.arm.setDesiredWristAngleDegrees(AbeConstants.WRIST_DEFAULT_ANGLE_DEGREES);
				}

				// look for chosen junction
				if(!slides){
					// get joystick direction
					Vector2d desiredAimVector = new Vector2d(junctionSelectionY, junctionSelectionX);

					// set chosen junction
					this.setChosenJunction(desiredAimVector);
				}

				// switch condition
				if (drop && !this.isEventScheduled(this.switchModeSchedule)) {
					// drop cone
					this.abe.arm.setHandUnclamped();

					// schedule switch
					this.switchModeSchedule = this.getScheduledTime(AIM_MODE_DEPOSIT_TIME_SECONDS);
				}

				// switch
				if(this.isEventFiring(this.switchModeSchedule)){
					// switch modes
					this.currentControlMode = ControlMode.GRABBING;

					// set up grab
					this.setupGrabMode();

					// unschedule event
					this.switchModeSchedule = UNSCHEDULED;
				}

				// update
				this.abe.update(true, true, !this.hasChosenJunction() || (this.hasChosenJunction() && slides));

				break;
			}
		}
	}

	public boolean hasChosenJunction(){
		return this.chosenJunction != null;
	}

	public void setChosenJunction(Vector2d direction){
		// get aim vector
		double distance = JUNCTION_AIM_DISTANCE_INCHES;

		// make sure it's actually aiming somewhere
		if(direction.norm() < 0.75){
			return;
		}

		// multiply by distance
		Vector2d aimVector = direction.times(distance);

		// add to pose estimate
		Vector2d chooseVector = this.abe.drive.getPoseEstimate().vec().plus(aimVector);

		// assign
		this.chosenJunction = chooseVector;
	}
}
