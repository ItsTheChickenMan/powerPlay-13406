package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.abe.AbeConstants;
import org.firstinspires.ftc.teamcode.lib.abe.AbeOpMode;
import org.firstinspires.ftc.teamcode.lib.utils.GlobalStorage;

@TeleOp(group = "AAA", name = "Initialize Abe")
public class InitializeAbe extends AbeOpMode {
	@Override
	public void runOpMode() throws InterruptedException {
		initialize(hardwareMap);

		GlobalStorage.clearGlobalStorage();

		waitForStart();

		this.abe.arm.setElbowAngleRadians(AbeConstants.ELBOW_INITIALIZATION_ANGLE_RADIANS);

		this.abe.arm.setSlidesRestingExtensionInches(AbeConstants.SLIDES_BASE_LENGTH_INCHES);

		this.abe.arm.update(true, false);

		while(!isStopRequested()){
			double elbowAngleRadians = this.abe.arm.getElbowAngleRadians();

			if(elbowAngleRadians > 0){
				GlobalStorage.currentElbowAngleRadians = elbowAngleRadians;
			}

			telemetry.addData("elbow angle (degrees)", this.abe.arm.getElbowAngleDegrees());
			telemetry.addData("initialize angle (degrees)", Math.toDegrees(AbeConstants.ELBOW_INITIALIZATION_ANGLE_RADIANS));
			telemetry.addData("global storage value (radians)", GlobalStorage.currentElbowAngleRadians);
			telemetry.update();
		}
	}
}
