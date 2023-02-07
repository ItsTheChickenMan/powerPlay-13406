package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.lib.abe.AbeAutonomous;

@Autonomous
public class RightPreloadPlusFive extends GenericAuto {
	@Override
	public void settings(){
		this.mode = Mode.RIGHT;
		this.depositAttempts = new int[]{6, 6, 6};
	}
}
