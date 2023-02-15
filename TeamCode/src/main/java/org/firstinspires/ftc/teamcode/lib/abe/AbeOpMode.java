package org.firstinspires.ftc.teamcode.lib.abe;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.lib.utils.JunctionHelper;

/**
 * @brief abstract class for writing an op mode that utilizes abe
 *
 * abe bot is technically just a robot equipped with the ability to aim to a large volume of points in space around it.  this class takes abe bot and applies it specifically to a Power Play opmode
 *
 * this class works like LinearOpMode in the sense that the opmode class must inherit from this class and implement runOpMode
 */
public abstract class AbeOpMode extends LinearOpMode {
	/**
	 * @brief Abe
	 */
	protected AbeBot abe;

	public void initialize(HardwareMap hardwareMap){
		// construct abe
		this.abe = new AbeBot(hardwareMap);
	}

	/**
	 * @param x
	 * @param y
	 * @return true if the coordinates are valid coordinates of a junction on the field, false if otherwise
	 */
	public boolean validateJunctionCoordinates(int x, int y){
		return x > 0 && x < 6 && y > 0 && y < 6;
	}

	/**
	 * @brief Aim to a junction using the junction coordinates provided
	 *
	 * Coordinates are measured from the bottom right corner of the field, starting at 0 (field perimeter) to 6 (field perimeter).
	 * Valid junctions are only at coordinates between 1 and 5.  Numbers outside of this range will make the method silently fail
	 *
	 * @param x x coordinate, from 1 to 5
	 * @param y y coordinate, from 1 to 5
	 */
	public void aimToJunction(int x, int y){
		// check coordinates
		if(!this.validateJunctionCoordinates(x, y)) return;

		// calculate junction coordinates
		Vector2d raw = JunctionHelper.snappedToRaw(x, y);

		// calculate junction height
		double height = JunctionHelper.getJunctionHeight(x, y);

		// modify height accordingly
		int index = JunctionHelper.getJunctionIndex(x, y);

		height += AbeConstants.JUNCTION_HEIGHT_OFFSETS_INCHES[index];

		// aim
		this.abe.aimAt(raw.getX(), height, raw.getY());
	}
}
