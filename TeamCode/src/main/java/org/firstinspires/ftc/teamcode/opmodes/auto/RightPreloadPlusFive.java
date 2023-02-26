package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class RightPreloadPlusFive extends GenericAuto {
	@Override
	public void settings(){
		MODE = Mode.RIGHT;
		DEPOSIT_ATTEMPTS = new int[]{6, 6, 6};
	}
}
