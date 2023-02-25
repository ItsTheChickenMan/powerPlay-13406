package org.firstinspires.ftc.teamcode.opmodes.teleop;

import android.content.Context;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.abe.AbeArm;
import org.firstinspires.ftc.teamcode.lib.abe.AbeConstants;
import org.firstinspires.ftc.teamcode.lib.abe.AbeOpMode;
import org.firstinspires.ftc.teamcode.lib.motion.LinearSlides;
import org.firstinspires.ftc.teamcode.lib.utils.JunctionHelper;

import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;

@Config
@TeleOp(group = "Tests", name = "Arm Tuner")
public class ArmTuner extends AbeOpMode {
	public static String FILENAME = "armtuning.txt";

	public static double ARM_RATE_INCHES = 2.0;
	public static double ANGULAR_CORRECTION_RATE_DEGREES = 1.0;

	@Override
	public void runOpMode() throws InterruptedException {
		Context context = hardwareMap.appContext;

		initialize(hardwareMap);

		// arraylist of points fetched
		ArrayList<Vector2d>[] points = new ArrayList[JunctionHelper.JUNCTION_LEVELS.length];

		for(int i = 0; i < points.length; i++){
			points[i] = new ArrayList<>();
		}

		// current level
		JunctionHelper.Level level = JunctionHelper.Level.LOW;

		// current angular correction
		double angularCorrectionRadians = 0.0;

		// current distance
		double armDistanceInches = 20;

		telemetry.addData("Status", "Initialized");
		telemetry.update();

		waitForStart();

		while(!isStopRequested() && !gamepad2.y){
			double delta = getDelta();
			resetDelta();

			int index = JunctionHelper.getJunctionIndex(level);

			// save point
			if(gamepadEx2.a_pressed){
				points[index].add(new Vector2d(armDistanceInches, angularCorrectionRadians));
			}

			// increase level
			if(gamepadEx2.dpad_up_pressed || gamepadEx2.dpad_down_pressed){
				if(gamepadEx2.dpad_up_pressed){
					index++;
				} else {
					index--;
				}

				index = Math.floorMod(index, JunctionHelper.JUNCTION_LEVELS.length);

				level = JunctionHelper.getJunctionLevelFromIndex(index);
			}

			// adjust arm distance
			armDistanceInches += -gamepad2.left_stick_x * ARM_RATE_INCHES * delta;

			// adjust angular correction
			angularCorrectionRadians += -gamepad2.right_stick_y * Math.toRadians(ANGULAR_CORRECTION_RATE_DEGREES) * delta;

			// get aim height
			double height = JunctionHelper.getJunctionHeight(level) - AbeConstants.ARM_Y_OFFSET_INCHES;

			// calculate aim values
			double elbowAngleRadians = this.abe.arm.calculateElbowAngleRadians(armDistanceInches, height) + angularCorrectionRadians;
			double slidesLengthInches = this.abe.arm.calculateSlidesLengthInches(armDistanceInches, height);

			// assign values
			this.abe.arm.setElbowAngleRadians(elbowAngleRadians);
			this.abe.arm.setSlidesLengthInches(slidesLengthInches);

			// update
			this.abe.arm.update();

			updateControllerStates();

			telemetry.addData("Level", level.toString());
			telemetry.addData("Height", JunctionHelper.getJunctionHeight(level));
			telemetry.addData("Distance", armDistanceInches);
			telemetry.addData("correction (radians)", angularCorrectionRadians);
			telemetry.addData("correction (degrees)", Math.toDegrees(angularCorrectionRadians));
			telemetry.addData("slides length", this.abe.arm.getSlidesLengthInches());
			telemetry.addData("elbow angle (degrees)", this.abe.arm.getElbowAngleDegrees());
			telemetry.addLine("press A to save a point, press Y to finish");
			telemetry.addData("number of points for " + level.toString(), points[index].size());
			telemetry.update();
		}

		// bring arm to neutral position
		this.abe.arm.setElbowAngleDegrees(0.0);
		this.abe.arm.setSlidesLengthInches(AbeConstants.SLIDES_BASE_LENGTH_INCHES + 0.5);

		this.abe.arm.update();

		Vector2d[][] staticPoints = new Vector2d[points.length][];

		// save values
		try {
			FileWriter writer = new FileWriter("/sdcard/FIRST/" + FILENAME);

			for(int i = 0; i < points.length; i++){
				ArrayList<Vector2d> p = points[i];

				staticPoints[i] = new Vector2d[p.size()];

				JunctionHelper.Level l = JunctionHelper.getJunctionLevelFromIndex(i);

				// clarifying comment
				writer.write("// " + l.toString() + "\n");

				// format with bracket
				writer.write("{");

				for(int j = 0; j < p.size(); j++) {
					Vector2d point = p.get(j);

					staticPoints[i][j] = point;

					// create constructor string for each vector
					writer.write("new Vector2d(" + point.getX() + ", " + point.getY() + "), ");
				}

				// closing bracket + double newline
				writer.write("},\n\n");
			}

			writer.close();
		} catch(IOException e){
			telemetry.addLine("error:");
			telemetry.addLine(e.getMessage());
			telemetry.addLine(e.getStackTrace().toString());
			telemetry.update();

			while(!isStopRequested());
		}

		// reload AbeArm to get R2 values
		AbeArm arm = new AbeArm(null, new LinearSlides(null, 0.0, 0.0, 0.0, 0.0), null, staticPoints);

		telemetry.addLine("Done, values saved to " + FILENAME);

		// load R2 values
		for(JunctionHelper.Level l : JunctionHelper.JUNCTION_LEVELS){
			telemetry.addData(l.toString() + " r2", arm.getSagCorrectionR2(l));
		}

		telemetry.update();

		while(!isStopRequested());
	}
}
