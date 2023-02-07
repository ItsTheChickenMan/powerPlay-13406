package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class LeftPreloadPlusTwo extends GenericAuto {
	@Override
	public void settings(){
		this.mode = Mode.LEFT;
		this.depositAttempts = new int[]{3, 3, 3};
	}
}
