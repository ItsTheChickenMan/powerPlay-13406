package org.firstinspires.ftc.teamcode.lib.abe;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.firstinspires.ftc.teamcode.lib.utils.JunctionHelper;
import org.firstinspires.ftc.teamcode.util.Encoder;

/**
 * @brief Abstract class containing required values and methods for using AbeBot during the Autonomous period
 *
 * This should only be for methods specifically tailored for Autonomous.  Methods that are tailored for TeleOp should be in AbeTeleOp, and methods for both should be in AbeOpMode
 *
 * Inherit from this class instead of Autonomous to use the methods
 */
public abstract class AbeAutonomous extends AbeOpMode {
	// constants //
	public enum Mode {
		LEFT,
		RIGHT
	}

	private enum CycleState {
		GRABBING,
		LIFTING,
		AIMING,
		EXTENDING,
		DEPOSITING,
		REORIENTING;

		public static final CycleState[] ORDERED_CYCLE_STATES = {GRABBING, LIFTING, AIMING, EXTENDING, DEPOSITING, REORIENTING};

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
					incremented = DEPOSITING;
					break;
				case DEPOSITING:
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
				case DEPOSITING:
					decremented = EXTENDING;
					break;
				case REORIENTING:
					decremented = DEPOSITING;
					break;
			}

			return decremented;
		}
	}

	public static final Vector2D CONE_STACK_RIGHT_POSITION = new Vector2D(36, 1.75);
	public static final Vector2D CONE_STACK_LEFT_POSITION = new Vector2D(AbeAutonomous.CONE_STACK_RIGHT_POSITION.getX(), AbeConstants.FIELD_SIZE_INCHES - CONE_STACK_RIGHT_POSITION.getY());

	private static final double BASE_CONE_STACK_HEIGHT = 3.0;
	private static final double CONE_STACK_HEIGHT_INCREASE_RATE = 1.375;
	private static final double SAFE_CONE_LIFTING_DISTANCE = 6.0;

	// states //

	// current mode
	private Mode mode;

	// what junction we're aiming at
	private Vector2D junction;

	// cone stack count relative to the current autonomous.  starts at 5
	private int coneStackCount = 5;

	// current cycle state
	private CycleState cycleState = CycleState.GRABBING;
	private double switchCycleSchedule = UNSCHEDULED;
	private int updatesSinceCycleSwitch = 0;

	// methods //

	public void setup(Mode mode, Vector2D junction){
		initializeAbe();

		this.mode = mode;
		this.junction = junction;
	}

	public void cycle(){
		// check current cycle state
		switch(this.cycleState){
			case GRABBING: {
				// aim at cone, wait until steady
				if(this.updatesSinceCycleSwitch == 1) {
					this.aimAtConeStack();
				}

				if(this.abe.isSteady() && !isEventScheduled(this.switchCycleSchedule)){
					// clamp
					this.abe.arm.clampFingers();

					// decrease cone stack size by 1
					this.coneStackCount--;

					// schedule switch
					this.switchCycleSchedule = getScheduledTime(0.5);
				}

				break;
			}

			case LIFTING: {
				// start aiming at junction with elbow
				if(this.updatesSinceCycleSwitch == 1){
					// FIXME: fix slides retracting too early to replace this hack
					this.abe.aimAt(this.getConeStackPosition().getX(), 16.0, this.getConeStackPosition().getY());

					//this.aimAtJunctionRaw(this.junction.getX(), this.junction.getY(), false, true, false);
				}

				telemetry.addData("arm height", this.abe.getArmHeight());

			// schedule switch as soon as elbow is safe distance upward
				if(this.abe.getArmHeight() > this.getTopConeStackHeight()+AbeAutonomous.SAFE_CONE_LIFTING_DISTANCE && !isEventScheduled(this.switchCycleSchedule)){
					this.switchCycleSchedule = getScheduledTime(0.0);
				}

				break;
			}

			case AIMING: {
				// start aiming at junction with drive as well
				if(this.updatesSinceCycleSwitch == 1){
					this.aimAtJunctionRaw(this.junction.getX(), this.junction.getY(), true, true, false);
				}

				// schedule switch when drive and arm are stable
				if(this.abe.isSteady() && !isEventScheduled(this.switchCycleSchedule)){
					this.switchCycleSchedule = getScheduledTime(0.0);
				}

				break;
			}

			case EXTENDING: {
				// aim with all three components
				if(this.updatesSinceCycleSwitch == 1) {
					this.aimAtJunctionRaw(this.junction.getX(), this.junction.getY(), true, true, true);
				}

				// schedule switch when drive and arm are stable
				if(this.abe.isSteady() && !isEventScheduled(this.switchCycleSchedule)){
					this.switchCycleSchedule = getScheduledTime(0.0);
				}

				break;
			}

			case DEPOSITING: {
				if(!isEventScheduled(this.switchCycleSchedule)){
					this.abe.arm.unclampFingers();

					// schedule switch after short time
					this.switchCycleSchedule = getScheduledTime(0.25);
				}

				break;
			}

			case REORIENTING: {
				// aim at stack with drive and elbow, but wait for slides
				if(this.updatesSinceCycleSwitch == 1){
					this.aimAtConeStack(true, true, false);
				}

				// switch when all is steady
				if(this.abe.isSteady() && !isEventScheduled(this.switchCycleSchedule)){
					this.switchCycleSchedule = getScheduledTime(0.0);
				}

				break;
			}
		}

		if(isScheduledEventHappening(this.switchCycleSchedule)){
			// increment cycle state
			this.cycleState = CycleState.increment(this.cycleState);

			// unschedule switch
			this.switchCycleSchedule = UNSCHEDULED;

			// reset counter
			this.updatesSinceCycleSwitch = 0;
		}

		this.updatesSinceCycleSwitch++;
	}

	public void update(){
		// update bot
		this.abe.update();
	}

	public int getConesInStack(){
		return this.coneStackCount;
	}

	public Vector2D getConeStackPosition(){
		return this.mode == Mode.LEFT ? AbeAutonomous.CONE_STACK_LEFT_POSITION : AbeAutonomous.CONE_STACK_RIGHT_POSITION;
	}

	public double getTopConeStackHeight(){
		// calculate height
		double height = AbeAutonomous.BASE_CONE_STACK_HEIGHT + AbeAutonomous.CONE_STACK_HEIGHT_INCREASE_RATE*this.coneStackCount;

		return height;
	}

	public void aimAtConeStack(){
		this.aimAtConeStack(true, true, true);
	}

	public void aimAtConeStack(boolean doDrive, boolean doElbow, boolean doSlides){
		double height = this.getTopConeStackHeight();

		// determine stack position from mode
		Vector2D stackPosition = this.getConeStackPosition();

		// aim
		this.abe.aimAt(stackPosition.getX(), height, stackPosition.getY(), doDrive, doElbow, doSlides);
	}
}
