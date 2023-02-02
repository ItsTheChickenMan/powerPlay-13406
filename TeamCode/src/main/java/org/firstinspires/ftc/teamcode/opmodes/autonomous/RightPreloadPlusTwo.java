package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class RightPreloadPlusTwo extends GenericRightSideAuto {
	@Override
	public void settings(){
		this.depositAttempts = new int[]{2, 2, 2};
	}
}
