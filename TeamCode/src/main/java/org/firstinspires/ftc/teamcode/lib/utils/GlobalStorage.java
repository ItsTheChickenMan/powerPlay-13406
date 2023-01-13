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

    // did we do auto?
    // generally going to be yes
    public static boolean didAuto;

    public static void clearGlobalStorage(){
        GlobalStorage.globalTelemetry = null;
        GlobalStorage.currentPose = new Pose2d();
        GlobalStorage.currentElbowAngleRadians = 0.0;
        GlobalStorage.currentSlidesExtension = AbeConstants.SLIDE_BASE_LENGTH_INCHES;
        GlobalStorage.didAuto = false;
    }

    public static void logGlobalStorage(Telemetry telemetry){
        telemetry.addData("globalTelemetry?", GlobalStorage.globalTelemetry != null);
        telemetry.addData("current pose", GlobalStorage.currentPose != null ? GlobalStorage.currentPose.toString() : "null");
        telemetry.addData("current elbow angle", GlobalStorage.currentElbowAngleRadians);
        telemetry.addData("current slides extension", GlobalStorage.currentSlidesExtension);
    }
}
