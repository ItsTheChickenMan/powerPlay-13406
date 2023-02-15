package org.firstinspires.ftc.teamcode.lib.abe;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class AbeConfig {
	public static class Hardware {
		// drive related hardware not included //

		// arm motors... //
		public DcMotorEx elbowMotor;
		public DcMotorEx slidesMotor;

		// arm servos... //
		public Servo wristServo;
		public Servo clawServo;

		public boolean complete(){
			// NOTE: I hate this method
			return this.elbowMotor != null &&
							this.slidesMotor != null &&
							this.wristServo != null &&
							this.clawServo != null
							;
		}
	}

	/**
	 * @brief Loads hardware from hardwareMap into an AbeConfig.Hardware class
	 *
	 * This is where to change the names of imports in hardware.  these should be used in every class and opmode involving abe
	 *
	 * @param hardwareMap
	 * @return
	 */
	public static AbeConfig.Hardware loadHardware(HardwareMap hardwareMap){
		AbeConfig.Hardware hardware = new AbeConfig.Hardware();

		hardware.elbowMotor = hardwareMap.get(DcMotorEx.class, "elbowMotor");
		hardware.slidesMotor = hardwareMap.get(DcMotorEx.class, "slidesMotor");

		hardware.wristServo = hardwareMap.get(Servo.class, "wristServo");
		hardware.clawServo = hardwareMap.get(Servo.class, "clawServo");

		return hardware;
	}
}