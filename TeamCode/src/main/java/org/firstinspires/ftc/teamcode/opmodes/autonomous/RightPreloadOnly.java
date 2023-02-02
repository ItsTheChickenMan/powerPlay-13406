package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class RightPreloadOnly extends GenericRightSideAuto {
	@Override
	public void settings(){
		this.depositAttempts = new int[]{0, 0, 0};
	}
}
