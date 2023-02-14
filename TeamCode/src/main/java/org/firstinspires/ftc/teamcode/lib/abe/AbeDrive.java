package org.firstinspires.ftc.teamcode.lib.abe;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class AbeDrive {
	// roadrunner drive
	private SampleMecanumDrive roadrunnerDrive;

	public AbeDrive(HardwareMap hardwareMap){
		this.roadrunnerDrive = new SampleMecanumDrive(hardwareMap);
	}

	public void update(){

	}
}
