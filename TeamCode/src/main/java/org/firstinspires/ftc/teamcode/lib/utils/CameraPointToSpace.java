package org.firstinspires.ftc.teamcode.lib.utils;

import com.vuforia.Vec3F;

import java.util.Vector;

/**
 * @brief Class for converting any point on an image to a point in space given many different values.
 *
 * This is fairly useless on its own, and is meant to be used alongside a system which is able to discern some of these values itself.
 */
public class CameraPointToSpace {
	private double g;
	private double h;

	public CameraPointToSpace(double distance, double width, double height){
		// calculate g constant (ratio of visible width to distance from camera)
		this.g = distance / width;
	}

	/**
	 * @brief Return the x displacement (in camera space) of a point on an image given a z displacement
	 *
	 * @param z displacement from the camera on the z axis (known)
	 * @param xPixel the horizontal displacement of the point on the image (treating 0 as the leftmost point)
	 * @param pixelWidth the pixel width of the image
	 * @return the x position of the point, in camera space (relative to the camera's position and rotation in space)
	 */
	public double xCameraDisplacement(double z, int xPixel, int pixelWidth){
		double x = z * xPixel/(this.g*pixelWidth);

		return x;
	}

	/**
	 * @brief Convert a point in camera space to a point in world space
	 *
	 * @param point
	 * @param cameraPosition
	 * @param cameraRotation
	 * @return the point in world space
	 */
	public Vec3F cameraToWorld(Vec3F point, Vec3F cameraPosition, Vec3F cameraRotation) {
		return new Vec3F();
	}

	/**
	 * @brief Return the x displacement (in real space) of a point on an image given a z displacement, a camera position and a camera rotation
	 *
	 * @note this will only use x and y values in cameraRotation and ignores any z value
	 *
	 * @param z
	 * @param xPixel
	 * @param pixelWidth
	 * @return the x displacement in
	 */
	public double xDisplacement(double z, int xPixel, int pixelWidth, Vec3F cameraPosition, Vec3F cameraRotation) {

		return 0.0;
	}
}
