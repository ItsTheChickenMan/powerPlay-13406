package org.firstinspires.ftc.teamcode.lib.utils;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;
import java.util.Map;

/**
 * @brief Class for managing "global" storage (data that persists between opmodes)
 */
public class GlobalStorage {
    // global telemetry instance
    public static Telemetry globalTelemetry;

    // FIXME: actually implement this cleanly
    //  for now, we'll just write each value manually because it's quicker for me to write
    public static Pose2d currentPose = new Pose2d(0, 0, 0);

    /**
     * @brief HashMap for storing numbers to string keys
     */
    //public static final Map<String, Double> numberStorage = new HashMap<String, Double>();
}
