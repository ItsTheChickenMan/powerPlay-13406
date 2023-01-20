package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.abe.AbeConstants;
import org.firstinspires.ftc.teamcode.lib.abe.AbeOpMode;
import org.firstinspires.ftc.teamcode.lib.utils.GlobalStorage;

@TeleOp(group = "Tests")
public class ArmStressTest extends AbeOpMode {
	@Override
	public void runOpMode() throws InterruptedException {
		GlobalStorage.globalTelemetry = telemetry;

		this.initializeAbe();

		telemetry.addData("Status", "Initialized");

		waitForStart();

		this.timer.reset();

		//double armX = AbeConstants.SLIDE_BASE_LENGTH_INCHES;
		//double armY = AbeConstants.ARM_VERTICAL_OFFSET_INCHES + AbeConstants.ELBOW_RADIUS_INCHES;
		double armX = 20;
		double armY = 40;
		double armZRotation = 0.0;

		double lastTime = this.timer.seconds();

		int state = 0;
		while( (!gamepad2.a && !gamepad2.b) && !isStopRequested()){
				if(gamepad2.a) state = 1;
				if(gamepad2.b) state = 2;
		}

		if(state == 1) {
			armY -= AbeConstants.ARM_VERTICAL_OFFSET_INCHES;

			this.abe.arm.enableManualControl();
			this.abe.arm.setElbowAngleRadians(this.abe.arm.calculateAimElbowAngleNoSagRadians(armX, armY), 25.0);
			this.abe.arm.extendSlidesTo(this.abe.arm.calculateAimSlidesLength(armX, armY), 15.0);

			while(!isStopRequested()) {
				if(gamepad2.right_bumper) break;

				telemetry.addData("state", "sin sag better");
				telemetry.addData("elbow angle", this.abe.arm.getElbowAngleDegrees());
				telemetry.update();
			}
		} else if(state == 2){
			while(!isStopRequested()){
				double currentTime = this.timer.seconds();
				double delta = currentTime - lastTime;
				lastTime = currentTime;

				double slowdown = 1.0;
				double changeRate = 3.0;

				slowdown /= changeRate;
				changeRate /= 1 + slowdown;
				changeRate *= 1 + slowdown - Math.max(gamepad2.left_trigger, gamepad2.right_trigger);

				armX -= gamepad2.left_stick_x * delta * changeRate;
				armY -= gamepad2.left_stick_y * delta * changeRate;

				// calculate armX + armZ
				double armZ = armX * Math.sin(armZRotation) - AbeConstants.ARM_LATERAL_OFFSET_INCHES;
				armX *= Math.cos(armZRotation);

				//this.abe.arm.aimAt(armX, armY - AbeConstants.ARM_VERTICAL_OFFSET_INCHES);
				this.abe.aimAt(armX + AbeConstants.ARM_LONGINAL_OFFSET_INCHES, armY, armZ);

				this.abe.update();

				//this.abe.arm.update();

				if(gamepad2.right_bumper) break;

				telemetry.addData("state", "con sag correct");
				telemetry.addData("x", armX);
				telemetry.addData("y", armY);
				telemetry.addData("angle", this.abe.arm.getElbowAngleDegrees());
				telemetry.addData("slides", this.abe.arm.getSlidesExtension());
				telemetry.addData("og elbow angle", Math.toDegrees(this.abe.arm.calculateAimElbowAngleNoSagRadians(armX, armY)));
				telemetry.addData("sag correction", Math.toDegrees(this.abe.arm.calculateSagCounter(this.abe.arm.calculateAimElbowAngleNoSagRadians(armX, armY), this.abe.arm.calculateAimSlidesLength(armX, armY), true)));
				telemetry.update();
			}
		}

		while(!gamepad2.right_bumper && !isStopRequested());

		this.abe.arm.enableManualControl();
		this.abe.arm.extendSlidesTo(16, 20);
		this.abe.arm.setElbowAngleDegrees(0.0, 20.0);

		while(!isStopRequested()){
			telemetry.addData("angle", this.abe.arm.getElbowAngleDegrees());
			telemetry.update();
		}
	}
}