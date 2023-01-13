package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.abe.AbeOpMode;
import org.firstinspires.ftc.teamcode.lib.utils.GlobalStorage;

/**
 * @brief brings abe to initialization position
 */
@TeleOp(name = "Initialize Abe", group = "AAA")
public class InitializeAbe extends AbeOpMode {
	public static final double STARTING_ANGLE_DEGREES = 38.6;

	@Override
	public void runOpMode() throws InterruptedException {
		// setup
		initializeAbe();

		// clear global storage
		GlobalStorage.clearGlobalStorage();

		this.abe.arm.enableManualControl();

		waitForStart();

		// bring elbow to starting angle
		this.abe.arm.setElbowAngleDegrees(STARTING_ANGLE_DEGREES, 30.0);

		while(!isStopRequested()){
			// update global storage elbow angle
			double a = this.abe.arm.getElbowAngleRadians();

			if(a > 0.1){
				GlobalStorage.currentElbowAngleRadians = a;
			}

			telemetry.addData("angle", this.abe.arm.getElbowAngleDegrees());
			telemetry.addData("global value", GlobalStorage.currentElbowAngleRadians);
			telemetry.update();
		};
	}
}
