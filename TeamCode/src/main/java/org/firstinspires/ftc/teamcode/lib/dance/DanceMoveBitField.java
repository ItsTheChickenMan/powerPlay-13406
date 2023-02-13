package org.firstinspires.ftc.teamcode.lib.dance;

/**
 * @brief Class for storing and managing single dance moves
 */
public class DanceMoveBitField {
	/*
	// DANCE BYTES
	// all of the bytes that each move occupies
	public static final long CLAW_BYTES = 0b1;
	public static final long SLIDES_BYTES = 0b1110;
	public static final long DRIVE_ROTATION_BYTES = 0b1110000;
	public static final long WRIST_BYTES = 0b110000000;
	public static final long DRIVE_TRANSLATION_BYTES = 0b111000000000;

	public static final long[] MOVE_BYTES = {DanceMoveBitField.CLAW_BYTES, DanceMoveBitField.SLIDES_BYTES, DanceMoveBitField.DRIVE_ROTATION_BYTES, DanceMoveBitField.WRIST_BYTES, DanceMoveBitField.DRIVE_TRANSLATION_BYTES};

	// DANCE MOVES
	public static void setClaw(AbeBot bot, int state){
		if(state == DanceMoveBitField.CLAW_CLOSED){
			bot.arm.clampFingers();
		} else {
			bot.arm.unclampFingers();
		}
	}

	public static void setSlides(AbeBot bot, int state){
		double extension = bot.arm.slides.getTravel() * 0.15 * (double)state + bot.arm.slides.getBaseExtension();

		bot.arm.slides.extendTo(extension, 30.0);
	}

	public static void setDriveRotation(AbeBot bot, int state){
		double angle = 45.0 * state;

		bot.drive.aimAtAngleDegrees(angle);
	}

	public static void setWrist(AbeBot bot, int state){
		double angle = 30.0 * state;

		bot.arm.wrist.rotateToDegrees(-bot.arm.wrist.getMaxRangeDegrees() / 2. + angle);
	}

	public static void setDriveTranslation(AbeBot bot, int state){
		// 0 = none
		// 1 = forward
		// 2 = backward
		// 3 = left
		// 4 = right
		double speed = 8.0;

		switch(state){
			case DRIVE_TRANSLATION_FORWARD:
				bot.drive.addToCumulativeVelocities(speed, speed, speed, speed);
				break;
			case DRIVE_TRANSLATION_BACKWARD:
				bot.drive.addToCumulativeVelocities(-speed, -speed, -speed, -speed);
				break;
			case DRIVE_TRANSLATION_LEFT:
				bot.drive.addToCumulativeVelocities(-speed, speed, speed, -speed);
				break;
			case DRIVE_TRANSLATION_RIGHT:
				bot.drive.addToCumulativeVelocities(speed, -speed, -speed, speed);
				break;
		}
	}

	interface DanceMove {
		void move(AbeBot bot, int state);
	}

	public static final DanceMove[] MOVES = new DanceMove[]{
					new DanceMove() {
						@Override
						public void move(AbeBot bot, int state) {
							setClaw(bot, state);
						}
					},
					new DanceMove() {
						@Override
						public void move(AbeBot bot, int state) {
							setSlides(bot, state);
						}
					},
					new DanceMove() {
						@Override
						public void move(AbeBot bot, int state) {
							setDriveRotation(bot, state);
						}
					},
					new DanceMove() {
						@Override
						public void move(AbeBot bot, int state) {
							setWrist(bot, state);
						}
					},
					new DanceMove() {
						@Override
						public void move(AbeBot bot, int state) {
							setDriveTranslation(bot, state);
						}
					}
	};

	public static int getLocation(long bytes){
		int count = 0;

		while(bytes % 2 == 0){
			bytes >>= 1;
			count++;
		}

		return count;
	}

	// DANCE STATES
	public static final int CLAW_CLOSED = 0;
	public static final int CLAW_OPEN = 1;

	public static final int SLIDES_000 = 0;
	public static final int SLIDES_015 = 1;
	public static final int SLIDES_030 = 2;
	public static final int SLIDES_045 = 3;
	public static final int SLIDES_060 = 4;
	public static final int SLIDES_075 = 5;
	public static final int SLIDES_090 = 6;

	public static final int DRIVE_ROTATION_000 = 0;
	public static final int DRIVE_ROTATION_045 = 1;
	public static final int DRIVE_ROTATION_090 = 2;
	public static final int DRIVE_ROTATION_135 = 3;
	public static final int DRIVE_ROTATION_180 = 4;
	public static final int DRIVE_ROTATION_225 = 5;
	public static final int DRIVE_ROTATION_270 = 6;
	public static final int DRIVE_ROTATION_315 = 7;

	public static final int WRIST_000 = 0;
	public static final int WRIST_030 = 1;
	public static final int WRIST_060 = 2;
	public static final int WRIST_090 = 3;

	public static final int DRIVE_TRANSLATION_NONE = 0;
	public static final int DRIVE_TRANSLATION_FORWARD = 1;
	public static final int DRIVE_TRANSLATION_BACKWARD = 2;
	public static final int DRIVE_TRANSLATION_LEFT = 3;
	public static final int DRIVE_TRANSLATION_RIGHT = 4;

	// current field
	private long field;

	public DanceMoveBitField(){
		this.field = 0;
	}

	public DanceMoveBitField add(long bytes, int state){
		int location = DanceMoveBitField.getLocation(bytes);

		// make sure state is right size
		int maxSize = (int)(bytes >> location);

		if(state > maxSize) return this;

		// shift state by b amount
		// TODO: also override sign byte? (can't remember which)
		state <<= location;

		// combine state with current field
		this.field |= state;

		// return self for chaining
		return this;
	}

	public int getState(long bytes){
		int location = DanceMoveBitField.getLocation(bytes);

		return (int)((this.field & bytes) >> location);
	}

	public void doMove(AbeBot bot){
		// loop through all moves
		for(int i = 0; i < DanceMoveBitField.MOVE_BYTES.length; i++){
			// get move
			long moveBytes = DanceMoveBitField.MOVE_BYTES[i];

			// get move state
			int moveState = this.getState(moveBytes);

			// do dance
			DanceMoveBitField.MOVES[i].move(bot, moveState);
		}
	}

	*/
}
