package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.abe.AbeConstants;
import org.firstinspires.ftc.teamcode.lib.abe.AbeOpMode;
import org.firstinspires.ftc.teamcode.lib.abe.AbeTeleOp;
import org.firstinspires.ftc.teamcode.lib.utils.JunctionHelper;
import org.firstinspires.ftc.teamcode.lib.utils.MathUtils;

@Config
@TeleOp(group = "Tests")
public class SagCorrectionTest extends AbeOpMode {
	public static double X_RATE = 2.0;
	public static double Y_RATE = 2.0;

	@Override
	public void runOpMode() throws InterruptedException {
		initialize(hardwareMap);

		telemetry.addData("Status", "Initialized");
		telemetry.update();

		waitForStart();

		double desiredX = AbeTeleOp.ARM_DEFAULT_POSITION_INCHES.getX();
		double desiredY = AbeTeleOp.ARM_DEFAULT_POSITION_INCHES.getY();

		JunctionHelper.Level level = JunctionHelper.Level.LOW;
		boolean usingLevel = false;

		while(!isStopRequested()){
			double dt = getDelta();
			resetDelta();

			boolean levelUp = gamepadEx2.dpad_up_pressed;
			boolean levelDown = gamepadEx2.dpad_down_pressed;

			if(levelUp || levelDown){
				usingLevel = true;
			}

			if(levelUp){
				level = JunctionHelper.incrementWrap(level);
			} else if(levelDown){
				level = JunctionHelper.decrementWrap(level);
			}

			double dx = -gamepad2.left_stick_x;
			double dy = -gamepad2.left_stick_y;

			if(!MathUtils.approximatelyEqual(dy, 0.0, 0.01)){
				usingLevel = false;
			}

			desiredX += dx * X_RATE * dt;

			if(usingLevel){
				// override desired y
				desiredY = JunctionHelper.getJunctionHeight(level);
			} else {
				desiredY += dy * Y_RATE * dt;
			}

			this.abe.arm.aimAt(desiredX, desiredY - AbeConstants.ARM_Y_OFFSET_INCHES);

			this.abe.arm.update();

			updateControllerStates();

			telemetry.addLine("Use the left joystick to move the arm forward/backward, the right joystick to move the arm up/down, and the dpad to snap the arm to a level");
			telemetry.addLine();
			telemetry.addData("desired x (forward/backward)", desiredX);
			telemetry.addData("desired y (up/down)", desiredY);
			telemetry.addData("elbow error (degrees)", this.abe.arm.getElbowAngleErrorDegrees());
			telemetry.addData("sag correction (degrees)", Math.toDegrees(this.abe.arm.calculateElbowSagCorrectionRadians(desiredX, desiredY)));

			telemetry.update();
		}
	}
}
