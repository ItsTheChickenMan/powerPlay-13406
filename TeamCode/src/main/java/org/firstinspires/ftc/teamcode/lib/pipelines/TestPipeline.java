package org.firstinspires.ftc.teamcode.lib.pipelines;

// using this class as a basic test pipeline for OpenCV.  Note that this is just for my own learning, and not for actually developing pipelines.

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class TestPipeline extends OpenCvPipeline {
	// "local" variables should go up here as instance variables, to avoid having to create Mats every process and to avoid leaks
	protected Mat grey = new Mat();

	// "analysis" results
	protected int analysis;

	@Override
	public void init(Mat firstFrame){
		// this calls once on the first frame received by the camera, and then never again
		// useful for mats which need to be initialized once with a frame
	}

	// process an image from input, and return a new image based on the processing
	@Override
	public Mat processFrame(Mat input) {
		Imgproc.cvtColor(input, grey, Imgproc.COLOR_RGB2GRAY);
		return grey;
	}

	// return analysis from a pipeline
	// useful for getting data from processFrame
	public int getAnalysis(){
		return analysis;
	}
}