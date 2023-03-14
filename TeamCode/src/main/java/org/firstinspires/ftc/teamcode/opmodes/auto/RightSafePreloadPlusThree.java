package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.lib.utils.PIDControllerRotation;

/**
 * Mimics the behavior of the auto we went to Feb 11th with.  Not subject to the pose drift of the other opmodes
 */
@Autonomous
public class RightSafePreloadPlusThree extends GenericAuto {
	@Override
	public void settings(){
		MODE = Mode.RIGHT;
		DEPOSIT_ATTEMPTS = new int[]{4, 4, 4};
		ROTATION_DIRECTION = PIDControllerRotation.RotationDirection.CW;
		LOCALIZATION_TYPE = SampleMecanumDrive.LocalizationType.TWO_WHEEL;
	}
}