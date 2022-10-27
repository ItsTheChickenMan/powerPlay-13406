package org.firstinspires.ftc.teamcode.lib.utils;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

/**
 * @brief Basic abstraction for the BNO055 imu
 */
public class Imu {
	private BNO055IMU imu;

	public Imu(BNO055IMU imu) {
		this.imu = imu;

		BNO055IMU.Parameters params = new BNO055IMU.Parameters();

		// params here...

		this.imu.initialize(params);
	}

	/**
	 * @brief returns angle of the imu in radians
	 *
	 * @return radians
	 */
	public float getAngleRadians(){
		return this.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
	}

	/**
	 * @brief returns angle of the imu in degrees
	 *
	 * @return degrees
	 */
	public float getAngleDegrees(){
		return this.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
	}
}
