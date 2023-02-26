package org.firstinspires.ftc.teamcode.lib.utils;

import org.firstinspires.ftc.teamcode.lib.pipelines.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;

import java.util.ArrayList;

/**
 * @brief abstracts the very advanced AprilTagDetectionPipeline and related settings into a simple class that's easy to use for our purposes
 */
public class SignalSleeveDetector {
	public static final int INVALID_ID = -1;

	private OpenCvCamera camera;
	private AprilTagDetectionPipeline aprilTagDetectionPipeline;

	// Lens intrinsics
	// UNITS ARE PIXELS
	// NOTE: stole this from the example class, don't plan on changing them since we don't really need localization
	private double fx = 578.272;
	private double fy = 578.272;
	private double cx = 402.145;
	private double cy = 221.506;

	// UNITS ARE METERS
	private double tagsize = 0.0317;

	// current spotted tag
	AprilTagDetection spottedTag;

	public static boolean isIdValid(int id){
		return id != SignalSleeveDetector.INVALID_ID;
	}

	public SignalSleeveDetector(OpenCvCamera camera){
		this.camera = camera;
		this.aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

		this.camera.setPipeline(this.aprilTagDetectionPipeline);
	}

	/**
	 * @brief If a tag is saved, clear that tag
	 *
	 * the search method doesn't override the saved spotted tag if there wasn't another tag found, so it doesn't clear it by default
	 */
	public void clearTag(){
		this.spottedTag = null;
	}

	/**
	 * @brief are we seeing/have we seen a tag?
	 *
	 * @return true if yes, false if no
	 */
	public boolean hasTag(){
		return this.spottedTag != null;
	}

	public int getSpottedTagId(){
		if(this.spottedTag == null)
			return INVALID_ID;

		return this.spottedTag.id;
	}

	/**
	 * @brief looks for any detections
	 */
	public void search(){
		// get latest detections from pipeline
		ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

		// find the closest april tag
		double distance = 99999999999999.0; // totally legit!

		if(currentDetections.size() != 0)
		{
			for(AprilTagDetection tag : currentDetections)
			{
				if(tag.pose.z < distance || this.spottedTag == null){
					this.spottedTag = tag;
					distance = tag.pose.z;
				}
			}
		}
	}
}