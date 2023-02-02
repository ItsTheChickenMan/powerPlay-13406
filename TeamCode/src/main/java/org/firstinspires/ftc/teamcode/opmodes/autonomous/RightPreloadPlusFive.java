package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class RightPreloadPlusFive extends GenericRightSideAuto {
	@Override
	public void settings(){
		this.depositAttempts = new int[]{5, 5, 5};
	}
}
