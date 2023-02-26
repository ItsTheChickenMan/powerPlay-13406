package org.firstinspires.ftc.teamcode.opmodes.teleop;

import android.provider.Settings;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.firstinspires.ftc.teamcode.lib.utils.GlobalStorage;

@TeleOp(name = "Set Auto Junction Offset", group = "AAA")
public class SetAutoJunctionOffset extends LinearOpMode {
	@Override
	public void runOpMode() throws InterruptedException {
		double rate = 0.1;

		boolean upLastFrame = false;
		boolean downLastFrame = false;
		boolean leftLastFrame = false;
		boolean rightLastFrame = false;

		while(!isStarted() && !isStopRequested()){
			boolean up = gamepad2.dpad_up;
			boolean down = gamepad2.dpad_down;
			boolean left = gamepad2.dpad_left;
			boolean right = gamepad2.dpad_right;

			Vector2d change = new Vector2d(up ? 1 : down ? -1 : 0, left ? 1 : right ? -1 : 0);

			if((up && !upLastFrame) || (down && !downLastFrame) || (left && !leftLastFrame) || (right && !rightLastFrame)){
				GlobalStorage.autoJunctionOffset = GlobalStorage.autoJunctionOffset.plus(change.times(rate));
			}

			upLastFrame = up;
			downLastFrame = down;
			leftLastFrame = left;
			rightLastFrame = right;

			GlobalStorage.logGlobalStorage(telemetry);
			telemetry.update();
		}
	}
}
