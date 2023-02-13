package org.firstinspires.ftc.teamcode.lib.abe;

import com.acmerobotics.dashboard.config.Config;

/**
 * @brief Contains all constant values required for Abe functionality.  All bot characteristics and settings should be here.
 *
 * Generally speaking, configurable constants should only be in here, and should only be used in classes in the abe package.
 */

// make configurable from dashboard
@Config
public class AbeConstants {
	// claw related...//
	public static double CLAW_CLOSED_ANGLE_DEGREES = 10;
	public static double CLAW_OPEN_ANGLE_DEGREES = -90;
}