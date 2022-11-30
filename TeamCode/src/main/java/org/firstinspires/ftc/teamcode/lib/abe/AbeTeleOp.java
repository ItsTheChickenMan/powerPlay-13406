package org.firstinspires.ftc.teamcode.lib.abe;

/**
 * @brief Abstract class containing required values and methods for using AbeBot during the Teleop period
 *
 * This should only be for methods specifically tailored for TeleOp.  Methods that are tailored for Autonomous should be in AbeAutonomous, and methods for both should be in AbeOpMode
 *
 * Inherit from this class instead of LinearOpMode to use the methods
 */
public abstract class AbeTeleOp extends AbeOpMode {
	/**
	 * @brief enum for the control mode
	 */
	public static enum ControlMode {
		GRABBING,
		AIMING
	}

	/**
	 * @brief Current control mode
	 */
	private AbeTeleOp.ControlMode mode;

	/**
	 * @brief set the current control mode
	 *
	 * @param mode
	 */
	public void setMode(AbeTeleOp.ControlMode mode){
		this.mode = mode;
	}

	public void update(){

	}
}
