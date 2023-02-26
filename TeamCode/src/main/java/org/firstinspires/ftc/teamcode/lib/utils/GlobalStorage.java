package org.firstinspires.ftc.teamcode.lib.utils;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
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
    public static Double currentElbowAngleRadians;

    // current slides extension
    public static Double currentSlidesExtension;

    // did we do auto?
    // generally going to be yes
    public static boolean didAuto;

    // offset to use for auto junction
    // manually set by the driver by eyeballing it before auto
    // not the best option, but not the worst either...just can't forget to do it (but if I do it defaults to 0, 0 so it should still be somewhat safe)
    public static Vector2d autoJunctionOffset;

    public static void clearGlobalStorage(){
        GlobalStorage.globalTelemetry = null;
        GlobalStorage.currentPose = new Pose2d();
        GlobalStorage.currentElbowAngleRadians = 0.0;
        GlobalStorage.currentSlidesExtension = AbeConstants.SLIDES_BASE_LENGTH_INCHES;
        GlobalStorage.didAuto = false;
        GlobalStorage.autoJunctionOffset = new Vector2d(0, 0);
    }

    public static void logGlobalStorage(Telemetry telemetry){
        telemetry.addData("globalTelemetry?", GlobalStorage.globalTelemetry != null);
        telemetry.addData("current pose", GlobalStorage.currentPose != null ? GlobalStorage.currentPose.toString() : "null");
        telemetry.addData("current elbow angle", GlobalStorage.currentElbowAngleRadians);
        telemetry.addData("current slides extension", GlobalStorage.currentSlidesExtension);
        telemetry.addData("auto junction offset", GlobalStorage.autoJunctionOffset != null ? GlobalStorage.autoJunctionOffset.toString() : "null");
        telemetry.addData("did auto", GlobalStorage.didAuto);
    }
}
