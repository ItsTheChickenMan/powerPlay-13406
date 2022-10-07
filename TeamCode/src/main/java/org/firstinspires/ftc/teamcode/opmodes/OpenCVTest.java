package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.lib.pipelines.TestPipeline;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

// basic opmode for testing opencv pipelines

@TeleOp
public class OpenCVTest extends LinearOpMode {
	// change this type to a different pipeline to test a different one
	TestPipeline pipeline = new TestPipeline();

	private OpenCvWebcam webcam;

	public void runOpMode(){
		// get camera stuff
		WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

		// streaming id
		int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

		// webcam
		webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

		webcam.setPipeline(pipeline);

		// TODO: deprecated
		webcam.openCameraDevice();

		webcam.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);

		// log, wait
		telemetry.addData("Status", "Initialized");
		telemetry.update();

		waitForStart();
	}
}
