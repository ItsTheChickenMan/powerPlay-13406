package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@Config
@TeleOp(group = "drive")
public class LocalizationTest extends LinearOpMode {
    public static double SPEED = 1.0;
    public static double LOCALIZATION_MODE = 1.0;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap, LOCALIZATION_MODE > 0.0 ? SampleMecanumDrive.LocalizationType.THREE_WHEEL : SampleMecanumDrive.LocalizationType.TWO_WHEEL);

        waitForStart();

        ElapsedTime timer = new ElapsedTime();
        int updates = 0;

        while (!gamepad2.a && !isStopRequested()) {
            Pose2d poseEstimate = drive.getPoseEstimate();

            Vector2d driveVector = new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x);

            //driveVector = driveVector.rotated(-poseEstimate.getHeading());

            drive.setWeightedDrivePower(
                    new Pose2d(
                            driveVector.getX(),
                            driveVector.getY(),
                            -gamepad1.right_stick_x
                    ).times(SPEED)
            );

            drive.update();

            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));
            telemetry.update();

            updates++;
        }

        double duration = timer.seconds();

        while(!isStopRequested()){
            telemetry.addData("ms / loop", (duration / updates) * 1000);
            telemetry.addData("loops / second", (updates / duration));
            telemetry.update();
        }
    }
}
