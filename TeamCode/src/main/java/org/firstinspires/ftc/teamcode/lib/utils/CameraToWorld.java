package org.firstinspires.ftc.teamcode.lib.utils;

import org.firstinspires.ftc.robotcore.internal.android.dex.util.ExceptionWithContext;

/**
 * @brief Class for converting any point on an image to a point in space given many different values.
 *
 * This is fairly useless on its own, and is meant to be used alongside a system which is able to discern some of these values itself.
 */
public class CameraToWorld {
	private int width;
	private int height;

	private double g;
	private double h;

	public CameraToWorld(int width, int height, double hDistance, double hWidth, double vDistance, double vHeight){
		this.width = width;
		this.height = height;

		// calculate g and h constants (ratio of visible width/height to z distance from camera)
		// TODO: this usually needs to be multiplied by width/height of image in pixels, but it might not be the best to do it here
		this.g = (hDistance / hWidth) * this.width;
		this.h = (vDistance / vHeight) * this.height;
	}

	/**
	 * @brief Return the x displacement (in camera space) of a point on an image given a z displacement
	 *
	 * @param z displacement from the camera on the z axis (known)
	 * @param xPixel the horizontal displacement of the point on the image
	 * @return the x position of the point, in camera space (relative to the camera's position and rotation in space)
	 */
	public double xCameraSpaceFromImage(double z, int xPixel) throws Exception {
		// translate x pixel to center
		xPixel -= this.width/2;

		if(xPixel > -this.width/2. || xPixel < -this.width/2.) throw new ExceptionWithContext("Invalid parameter xPixel");

		return (z * xPixel) / this.g;
	}

	public double yCameraSpaceFromImage(double z, int yPixel){
		// translate y pixel to center
		yPixel -= this.height/2;

		if(yPixel > -this.height/2. || yPixel < -this.height/2.) throw new ExceptionWithContext("Invalid parameter yPixel");

		return (z * yPixel) / this.h;
	}

	public void zCamera(double x, int xPixel) {

	}
}
