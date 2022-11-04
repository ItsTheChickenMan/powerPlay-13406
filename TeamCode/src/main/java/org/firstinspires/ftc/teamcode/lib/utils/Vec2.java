package org.firstinspires.ftc.teamcode.lib.utils;

/**
 * @brief Dead simple and stupid vector implementation
 */
public class Vec2 {
	public double x;
	public double y;

	public Vec2(){
		this.x = 0;
		this.y = 0;
	}

	public Vec2(double num) {
		this.x = num;
		this.y = num;
	}

	public Vec2(double x, double y){
		this.x = x;
		this.y = y;
	}

	// static methods //
	public static Vec2 add(Vec2 v1, Vec2 v2){
		return new Vec2(v1.x + v2.x, v1.y + v2.y);
	}

	public static Vec2 sub(Vec2 v1, Vec2 v2){
		return new Vec2(v1.x - v2.x, v1.y - v2.y);
	}

	public static Vec2 mult(Vec2 v1, double d) {
		return new Vec2(v1.x*d, v1.y*d);
	}

	public static double dot(Vec2 v1, Vec2 v2){
		return v1.x*v2.x + v1.y*v2.y;
	}

	public static Vec2 rotate(Vec2 v1, double a){
		double sn = Math.sin(a);
		double cs = Math.cos(a);

		double x = v1.x * cs - v1.y * sn;
		double y = v1.x * sn + v1.y * cs;

		return new Vec2(x, y);
	}

	// non-static
	public void add(Vec2 v1){
		this.x += v1.x;
		this.y += v1.y;
	}

	public void sub(Vec2 v1){
		this.x -= v1.x;
		this.y -= v1.y;
	}

	public void mult(double d){
		this.x *= d;
		this.y *= d;
	}

	public double dot(Vec2 v1){
		return this.x*v1.x + this.y*v1.y;
	}

	public void rotate(double a){
		double sn = Math.sin(a);
		double cs = Math.cos(a);

		double x = this.x * cs - this.y * sn;
		double y = this.x * sn + this.y * cs;

		this.x = x;
		this.y = y;
	}
}