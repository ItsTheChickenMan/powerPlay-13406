package org.firstinspires.ftc.teamcode.lib.abe;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class AbeConfig {
	public static AbeBot.Hardware loadHardware(HardwareMap hardwareMap){
		AbeBot.Hardware hardware = new AbeBot.Hardware();

		hardware.frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
		hardware.frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
		hardware.backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
		hardware.backRight = hardwareMap.get(DcMotorEx.class, "backRight");

		//this.hardware.frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
		//hardware.frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
		//this.hardware.backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
		//hardware.backRight.setDirection(DcMotorSimple.Direction.REVERSE);

		hardware.elbow = hardwareMap.get(DcMotorEx.class, "angleMotor");
		hardware.slides = hardwareMap.get(DcMotorEx.class, "slidesMotor");
		hardware.slides2 = hardwareMap.get(DcMotorEx.class, "slides2");

		hardware.elbow.setDirection(DcMotorSimple.Direction.REVERSE);
		hardware.slides.setDirection(DcMotorSimple.Direction.REVERSE);

		hardware.wristServo = hardwareMap.get(Servo.class, "wristServo");
		hardware.fingerServo = hardwareMap.get(Servo.class, "fingerServo");

		hardware.elbowLimitSensor = hardwareMap.get(TouchSensor.class, "limitSwitch");

		hardware.imu = hardwareMap.get(BNO055IMU.class, "imu");

		return hardware;
	}
}
