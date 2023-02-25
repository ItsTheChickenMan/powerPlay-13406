package org.firstinspires.ftc.teamcode.lib.utils;

import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * @brief gamepad, but supports single presses and such (I think it's referred to as a rising edge detector or something)
 * 
 * This is not meant to replace the gamepad, but you can use it alongside of it or use the normal gamepad for normal gamepad reasons
 */
public class GamepadEx {
	private Gamepad gamepad;

	// controller states cache //

	// ---- PRESSED ---- //
	public boolean dpad_up_pressed = false;
	public boolean dpad_down_pressed = false;
	public boolean dpad_left_pressed = false;
	public boolean dpad_right_pressed = false;
	public boolean a_pressed = false;
	public boolean b_pressed = false;
	public boolean x_pressed = false;
	public boolean y_pressed = false;
	public boolean start_pressed = false;
	public boolean back_pressed = false;
	public boolean left_bumper_pressed = false;
	public boolean right_bumper_pressed = false;

	// ---- TOGGLED ---- //
	public boolean dpad_up_toggled = false;
	public boolean dpad_down_toggled = false;
	public boolean dpad_left_toggled = false;
	public boolean dpad_right_toggled = false;
	public boolean a_toggled = false;
	public boolean b_toggled = false;
	public boolean x_toggled = false;
	public boolean y_toggled = false;
	public boolean start_toggled = false;
	public boolean back_toggled = false;
	public boolean left_bumper_toggled = false;
	public boolean right_bumper_toggled = false;
	
	// ---- LAST ---- //
	private boolean dpad_up_last = false;
	private boolean dpad_down_last = false;
	private boolean dpad_left_last = false;
	private boolean dpad_right_last = false;
	private boolean a_last = false;
	private boolean b_last = false;
	private boolean x_last = false;
	private boolean y_last = false;
	private boolean start_last = false;
	private boolean back_last = false;
	private boolean left_bumper_last = false;
	private boolean right_bumper_last = false;


	public GamepadEx(Gamepad gamepad){
		this.gamepad = gamepad;
	}

	public void updateControllerStates(){
		// load states
		boolean dpad_up = this.gamepad.dpad_up;
		boolean dpad_down = this.gamepad.dpad_down;
		boolean dpad_left = this.gamepad.dpad_left;
		boolean dpad_right = this.gamepad.dpad_right;

		boolean a = this.gamepad.a;
		boolean b = this.gamepad.b;
		boolean y = this.gamepad.y;
		boolean x = this.gamepad.x;

		boolean left_bumper = this.gamepad.left_bumper;
		boolean right_bumper = this.gamepad.right_bumper;

		boolean start = this.gamepad.start;
		boolean back = this.gamepad.back;


		// pressed

		// dpad
		this.dpad_up_pressed = dpad_up && !this.dpad_up_last;
		this.dpad_down_pressed = dpad_down && !this.dpad_down_last;
		this.dpad_left_pressed = dpad_left && !this.dpad_left_last;
		this.dpad_right_pressed = dpad_right && !this.dpad_right_last;

		// letters
		this.a_pressed = a && !this.a_last;
		this.b_pressed = b && !this.b_last;
		this.y_pressed = y && !this.y_last;
		this.x_pressed = x && !this.x_last;

		// bumpers
		this.left_bumper_pressed = left_bumper && !this.left_bumper_last;
		this.right_bumper_pressed = right_bumper && !this.right_bumper_last;

		// start + back
		this.start_pressed = start && !this.start_last;
		this.back_pressed = back && !this.back_last;


		// toggled

		// dpad
		this.dpad_up_toggled = this.dpad_up_pressed != this.dpad_up_toggled;
		this.dpad_down_toggled = this.dpad_down_pressed != this.dpad_down_toggled;
		this.dpad_left_toggled = this.dpad_left_pressed != this.dpad_left_toggled;
		this.dpad_right_toggled = this.dpad_right_pressed != this.dpad_right_toggled;

		// letters
		this.a_toggled = this.a_pressed != this.a_toggled;
		this.b_toggled = this.b_pressed != this.b_toggled;
		this.y_toggled = this.y_pressed != this.y_toggled;
		this.x_toggled = this.x_pressed != this.x_toggled;

		// bumpers
		this.left_bumper_toggled = this.left_bumper_pressed != this.left_bumper_toggled;
		this.right_bumper_pressed = this.right_bumper_pressed != this.right_bumper_toggled;

		// start + back
		this.start_pressed = this.start_pressed != this.start_toggled;
		this.back_pressed = this.back_pressed != this.back_toggled;


		// save last states

		// dpad
		this.dpad_up_last = dpad_up;
		this.dpad_down_last = dpad_down;
		this.dpad_left_last = dpad_left;
		this.dpad_right_last = dpad_right;

		// letters
		this.a_last = a;
		this.b_last = b;
		this.y_last = y;
		this.x_last = x;

		// bumpers
		this.left_bumper_last = left_bumper;
		this.right_bumper_last = right_bumper;

		// start + back
		this.start_last = start;
		this.back_last = back;
	}
}
