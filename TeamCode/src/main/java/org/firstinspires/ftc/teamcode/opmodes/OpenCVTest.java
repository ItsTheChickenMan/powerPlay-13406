package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp
public class OpenCVTest extends LinearOpMode {
	private OpenCvWebcam webcam;

	public void runOpMode(){
		// get camera stuff
		WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

		// streaming id
		int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

		// webcam
		webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

		webcam.openCameraDevice();

		webcam.startStreaming(1280, 720, OpenCvCameraRotation.SIDEWAYS_LEFT);

		// log, wait
		telemetry.addData("Status", "Initialized");
		telemetry.update();

		waitForStart();

	}
}
