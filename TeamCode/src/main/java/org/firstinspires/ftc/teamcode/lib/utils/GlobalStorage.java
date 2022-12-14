package org.firstinspires.ftc.teamcode.lib.utils;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.abe.AbeConstants;

/**
 * @brief Class for managing "global" storage (data that persists between opmodes)
 */
public class GlobalStorage {
    // global telemetry instance
    public static Telemetry globalTelemetry;

    // current pose
    public static Pose2d currentPose;

    // current elbow angle
    public static double currentElbowAngleRadians;

    // current slides extension
    public static double currentSlidesExtension;

    public static void clearGlobalStorage(){
        GlobalStorage.globalTelemetry = null;
        GlobalStorage.currentPose = new Pose2d();
        GlobalStorage.currentElbowAngleRadians = 0.0;
        GlobalStorage.currentSlidesExtension = AbeConstants.SLIDE_BASE_LENGTH_INCHES;
    }

    /**
     * @brief HashMap for storing numbers to string keys
     */
    //public static final Map<String, Double> numberStorage = new HashMap<String, Double>();
}
