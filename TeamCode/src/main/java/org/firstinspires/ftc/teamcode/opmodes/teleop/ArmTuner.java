package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.abe.AbeConstants;
import org.firstinspires.ftc.teamcode.lib.abe.AbeOpMode;
import org.firstinspires.ftc.teamcode.lib.abe.AbeTeleOp;
import org.firstinspires.ftc.teamcode.lib.utils.JunctionHelper;

@TeleOp(group = "Tests")
public class ArmTuner extends AbeOpMode {
	@Override
	public void runOpMode() throws InterruptedException {
		this.initializeAbe();

		waitForStart();

		JunctionHelper.Level currentLevel = JunctionHelper.Level.GROUND;

		double desiredCorrection = 0.0;

		int armXIndex = 0;

		boolean pressedLastFrame = false;

		this.abe.arm.enableManualControl();

		double[][] armOffsets = {new double[4], new double[4], new double[4], new double[4]};

		ElapsedTime timer = new ElapsedTime();

		double lastTime = timer.seconds();

		while(!isStopRequested()){
			double currentTime = timer.seconds();
			double delta = currentTime - lastTime;
			lastTime = currentTime;

			double armX = armXIndex*6.0 + 16.0;
			double armY = JunctionHelper.getJunctionHeight(currentLevel) - AbeConstants.ARM_VERTICAL_OFFSET_INCHES + AbeConstants.ARM_POLE_HEIGHT_OFFSET_INCHES;

			double trueElbowAngle = this.abe.arm.calculateAimElbowAngleNoSagRadians(armX, armY);
			double slidesLength = this.abe.arm.calculateAimSlidesLength(armX, armY);

			this.abe.arm.setElbowAngleRadians(trueElbowAngle + desiredCorrection, Math.toRadians(20.0));
			this.abe.arm.extendSlidesTo(slidesLength, 10.0);

			if(gamepad2.a && !pressedLastFrame){
				// save offset
				armOffsets[levelToInt(currentLevel)][armXIndex] = desiredCorrection;

				// update offset
				armXIndex++;

				if(armXIndex > 3){
					armXIndex = 0;

					currentLevel = this.incrementLevel(currentLevel);

					if(currentLevel == JunctionHelper.Level.NONE) break;
				}
			}

			pressedLastFrame = gamepad2.a;

			desiredCorrection -= Math.toRadians(gamepad2.left_stick_y * delta);

			telemetry.addLine("tune the arm to the proper height using joystick and then press A");
			telemetry.addData("currentLevel", this.levelToString(currentLevel));
			telemetry.addData("desired distance", armX);
			telemetry.addData("arm height", armY);
			telemetry.addData("desired height", armY + AbeConstants.ARM_VERTICAL_OFFSET_INCHES);
			telemetry.addData("correction (degrees)", Math.toDegrees(desiredCorrection));
			telemetry.update();
		}

		while(!isStopRequested()){
			for(int i = 0; i < armOffsets.length; i++){
				for(int j = 0; j < armOffsets[i].length; j++){
					telemetry.addData("level", i);
					telemetry.addData("height", j*6 + 16);
					telemetry.addData("offset", armOffsets[i][j]);
					telemetry.addLine("");
				}
			}

			telemetry.update();
		}
	}

	public String levelToString(JunctionHelper.Level level){
		if(level == JunctionHelper.Level.GROUND){
			return "GROUND";
		} else if(level == JunctionHelper.Level.LOW){
			return "LOW";
		} else if(level == JunctionHelper.Level.MEDIUM){
			return "MEDIUM";
		} else if(level == JunctionHelper.Level.HIGH){
			return "HIGH";
		}

		return "NONE";
	}

	public JunctionHelper.Level incrementLevel(JunctionHelper.Level level){
		if(level == JunctionHelper.Level.GROUND){
			return JunctionHelper.Level.LOW;
		} else if(level == JunctionHelper.Level.LOW){
			return JunctionHelper.Level.MEDIUM;
		} else if(level == JunctionHelper.Level.MEDIUM){
			return JunctionHelper.Level.HIGH;
		}

		return JunctionHelper.Level.NONE;
	}

	public int levelToInt(JunctionHelper.Level level){
		if(level == JunctionHelper.Level.GROUND){
			return 0;
		} else if(level == JunctionHelper.Level.LOW){
			return 1;
		} else if(level == JunctionHelper.Level.MEDIUM){
			return 2;
		} else if(level == JunctionHelper.Level.HIGH){
			return 3;
		}

		return -1;
	}
}
