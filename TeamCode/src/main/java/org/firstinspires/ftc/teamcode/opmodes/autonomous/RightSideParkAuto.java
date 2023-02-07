package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import android.provider.Settings;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.firstinspires.ftc.teamcode.lib.abe.AbeAutonomous;
import org.firstinspires.ftc.teamcode.lib.abe.AbeDrive;
import org.firstinspires.ftc.teamcode.lib.utils.AprilTagDetector;
import org.firstinspires.ftc.teamcode.lib.utils.GlobalStorage;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class RightSideParkAuto extends GenericAuto {
	@Override
	public void settings(){
		this.mode = Mode.RIGHT;
		this.depositAttempts = new int[]{0, 0, 0};
	}
}
