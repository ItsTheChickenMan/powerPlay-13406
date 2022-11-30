package org.firstinspires.ftc.teamcode.lib.abe;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * @brief Abstract class containing required values and methods for using AbeBot in a general OpMode
 *
 * More specific methods for autonomous and teleop are contained in AbeAutonomous and AbeTeleOp, respectively
 */
public abstract class AbeOpMode extends LinearOpMode {
	// abe bot
	protected AbeBot abe;

	/**
	 * @brief Log an error to telemetry and freeze the program until stopped
	 */
	public void logError(String errorMessage){
		telemetry.addData("error", errorMessage);
		telemetry.update();

		while(!isStopRequested());
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
}
