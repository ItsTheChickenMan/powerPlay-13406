package org.firstinspires.ftc.teamcode.lib.abe;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.lib.utils.AngleHelper;
import org.firstinspires.ftc.teamcode.lib.utils.JunctionHelper;
import org.firstinspires.ftc.teamcode.lib.utils.MathUtils;

@Config
public abstract class AbeTeleOp extends AbeOpMode {
	// constants
	public static double GRAB_MODE_PICKUP_TIME_SECONDS = 0.25;
	public static double AIM_MODE_DEPOSIT_TIME_SECONDS = 0.1;

	public static Vector2d ARM_DEFAULT_POSITION_INCHES = new Vector2d(13, 15);
	public static Vector2d ARM_GRAB_POSITION_INCHES = new Vector2d(16.5, 2.3);
	public static double ARM_EXTENSION_INCHES = 12.0;

	public static double MIN_POSE_CORRECTION_RATE_INCHES_PER_SECOND = 8.0;
	public static double POSE_CORRECTION_RATE_MULTIPLIER = 1.75;
	public static double MAX_POSE_CORRECTION_RATE_INCHES_PER_SECOND = MIN_POSE_CORRECTION_RATE_INCHES_PER_SECOND * POSE_CORRECTION_RATE_MULTIPLIER;

	public static double JUNCTION_AIM_DISTANCE_INCHES = 16.0;

	public static double WRIST_RAISE_THRESHOLD_RADIANS = Math.toRadians(45.0);

	public static double REACH_DOWN_VERTICAL_OFFSET = 1.0;

	public static double MIN_POWER = 0.2;
	public static double MAX_POWER = 0.6;
	public static double ROTATION_POWER = 0.5;

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
	private int chosenJunctionIndex;

	// schedules
	private double switchModeSchedule = UNSCHEDULED;

	// methods
	public int getChosenJunctionIndex(){
		return this.chosenJunctionIndex;
	}

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
		double speedTrigger = Math.max(gamepad1.left_trigger, gamepad1.right_trigger);
		double rate = MathUtils.map(speedTrigger, 0.0, 1.0, MAX_POWER, MIN_POWER);
		double forward = -gamepad1.left_stick_y;
		double strafe = -gamepad1.left_stick_x;
		// aim vector for drive rotation
		Vector2d rotationVector = new Vector2d(-gamepad1.right_stick_y, -gamepad1.right_stick_x);
		double rotate = this.getDesiredDriveRotation(rotationVector, rate);

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
				boolean grabExtension = gamepad2.left_trigger > 0.05;
				boolean reachDown = gamepad2.left_bumper;

				// default aim angle
				this.abe.arm.setAimWristAngleDegrees(AbeConstants.WRIST_GRAB_ANGLE_DEGREES);

				double horizontalExtension = ARM_GRAB_POSITION_INCHES.getX() + (grabExtension ? ARM_EXTENSION_INCHES : 0);

				if(!down) {
					// default to ground mode
					this.currentGrabMode = GrabMode.GROUND;

					// move to default position
					this.abe.arm.aimAt(ARM_DEFAULT_POSITION_INCHES.getX() + AbeConstants.WRIST_OFFSET_INCHES, ARM_DEFAULT_POSITION_INCHES.getY() - AbeConstants.ARM_Y_OFFSET_INCHES);

					// wrist down
					this.abe.arm.setDesiredWristAngleDegrees(AbeConstants.WRIST_DEFAULT_LOW_ANGLE_DEGREES);
				} else {
					double verticalOffset = 0.0;

					if(!reachDown) {
						// angle the wrist slightly down to make grabbing easier
						this.abe.arm.setDesiredWristAngleDegrees(AbeConstants.WRIST_GRAB_ANGLE_DEGREES);
					} else {
						// angle the wrist down fully to grab dropped cones
						this.abe.arm.setAimWristAngleDegrees(AbeConstants.WRIST_LOW_GRAB_ANGLE_DEGREES);
						this.abe.arm.setDesiredWristAngleDegrees(AbeConstants.WRIST_LOW_GRAB_ANGLE_DEGREES);

						verticalOffset = REACH_DOWN_VERTICAL_OFFSET;
					}

					// check grab mode
					switch (this.currentGrabMode) {
						case GROUND: {
							// go to down position
							this.abe.arm.aimAt(horizontalExtension + AbeConstants.WRIST_OFFSET_INCHES, ARM_GRAB_POSITION_INCHES.getY() - AbeConstants.ARM_Y_OFFSET_INCHES + verticalOffset);

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
							this.abe.arm.aimAt(horizontalExtension + AbeConstants.WRIST_OFFSET_INCHES, grabHeight - AbeConstants.ARM_Y_OFFSET_INCHES);

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

					// clamp
					this.abe.arm.setHandClamped();
				}

				// switch
				if(isEventFiring(this.switchModeSchedule)){
					// switch mode
					this.currentControlMode = ControlMode.AIMING;

					// unschedule event
					this.switchModeSchedule = UNSCHEDULED;

					// setup aim
					this.setupAimMode();
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
				boolean deselectJunction = gamepad2.x;

				double junctionSelectionX = -gamepad2.left_stick_x;
				double junctionSelectionY = -gamepad2.left_stick_y;

				double poseCorrectionX = gamepad2.right_stick_y;
				double poseCorrectionY = gamepad2.right_stick_x;
				double correctionSpeed = gamepad2.right_trigger;

				boolean raise = false;

				// check chosen junction
				if(hasChosenJunction()) {
					// check slides
					if(slides){
						// check pose correction
						Vector2d poseCorrection = new Vector2d(poseCorrectionX, poseCorrectionY);

						if(poseCorrection.norm() > 0.2){
							// check if direction that pose correction is pointing and robot's forward are away from each other
							double poseAngle = poseCorrection.angle();
							double forwardAngle = this.abe.drive.getPoseEstimate().getHeading();

							raise = MathUtils.approximatelyEqual(poseAngle, forwardAngle, WRIST_RAISE_THRESHOLD_RADIANS);
						}

						// correction rate
						double min = MIN_POSE_CORRECTION_RATE_INCHES_PER_SECOND;
						double max = MAX_POSE_CORRECTION_RATE_INCHES_PER_SECOND;
						double correctionRate = MathUtils.map(correctionSpeed, 0.0, 1.0, min, max);

						correctionRate *= delta;

						poseCorrection = poseCorrection.times(correctionRate);

						this.abe.drive.addToPoseOffset(poseCorrection);
					}

					// aim
					this.abe.arm.setAimWristAngleDegrees(AbeConstants.WRIST_UP_ANGLES_DEGREES[getChosenJunctionIndex()]);
					aimToJunctionRaw(this.chosenJunction);
				} else {
					// clear aim
					this.abe.clearAim();

					// aim at default
					this.abe.arm.aimAt(ARM_DEFAULT_POSITION_INCHES.getX() + AbeConstants.WRIST_OFFSET_INCHES, ARM_DEFAULT_POSITION_INCHES.getY() - AbeConstants.ARM_Y_OFFSET_INCHES);

					this.abe.arm.setAimWristAngleDegrees(AbeConstants.WRIST_GRAB_ANGLE_DEGREES);
					this.abe.arm.setDesiredWristAngleDegrees(AbeConstants.WRIST_DEFAULT_HIGH_ANGLE_DEGREES);
				}

				// check wrist
				if(wristing){
					this.abe.arm.setDesiredWristAngleDegrees(AbeConstants.WRIST_DROP_ANGLES_DEGREES[getChosenJunctionIndex()]);
				} else if(raise) {
					this.abe.arm.setDesiredWristAngleDegrees(AbeConstants.WRIST_HIGH_ANGLE_DEGREES);
				} else {
					this.abe.arm.setDesiredWristAngleDegrees(AbeConstants.WRIST_UP_ANGLES_DEGREES[getChosenJunctionIndex()]);
				}

				if(deselectJunction){
					this.chosenJunction = null;
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

		// also assign
		this.chosenJunctionIndex = JunctionHelper.getJunctionIndex(JunctionHelper.getJunctionLevelFromRaw(this.chosenJunction.getX(), this.chosenJunction.getY()));

		// why does this even happen???
		if(this.chosenJunctionIndex == -1){
			this.chosenJunctionIndex = 0;
		}
	}

	public double getDesiredDriveRotation(Vector2d aimVector, double speed){
		if(aimVector.norm() < 0.25) return 0.0;

		// get rotational power
		double rotationSpeed = ROTATION_POWER * speed;

		double aimVectorRotation = aimVector.angle();
		double forwardVectorRotation = this.abe.drive.getPoseEstimate().getHeading();

		// GlobalStorage.globalTelemetry.addData("rotation speed", speed);
		// GlobalStorage.globalTelemetry.addData("aimVector", aimVector);
		// GlobalStorage.globalTelemetry.addData("aimVectorRotation", aimVectorRotation);
		// GlobalStorage.globalTelemetry.addData("forwardVectorRotation", forwardVectorRotation);

		// get direction
		double distance = AngleHelper.angularDistanceRadians(forwardVectorRotation, aimVectorRotation);

		// GlobalStorage.globalTelemetry.addData("angular distance", distance);

		return distance * rotationSpeed;
	}
}
