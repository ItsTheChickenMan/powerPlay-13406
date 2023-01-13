package org.firstinspires.ftc.teamcode.lib.dance;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.lib.abe.AbeBot;

import java.util.ArrayList;
import java.util.HashMap;

/**
 * @brief class for creating and running dance sequences
 */
public class DanceSequence {
	/*
	// each dance move in sequence
	private ArrayList<DanceMoveBitField> sequence;

	// number of beats to wait until the next dance move, for each move in the sequence
	private ArrayList<Integer> beats;

	// all defined dance moves
	private HashMap<String, DanceMoveBitField> moves;

	// beat keeper
	private BeatKeeper beatKeeper;

	public DanceSequence(double bpm){
		this.sequence = new ArrayList<DanceMoveBitField>();
		this.beats = new ArrayList<Integer>();
		this.moves = new HashMap<String, DanceMoveBitField>();
		this.beatKeeper = new BeatKeeper(bpm);
	}

	/**
	 * @brief Give a simplified string name to a complex dance move
	 *
	 * Useful for moves which are frequently repeated
	 *
	 * @param name name to be used to reference this move while building the sequence
	 * @param move the DanceMoveBitField for this move
	 * @return self for chaining
	 *
	public DanceSequence defineMove(String name, DanceMoveBitField move){
		// add to moves
		this.moves.put(name, move);

		// return self for chaining
		return this;
	}

	/**
	 * @brief add a new defined move to the dance sequence
	 *
	 * @param name name of the defined move
	 * @param beats beats to wait until the next move
	 * @return self for chaining
	 *
	public DanceSequence addMoveToSequence(String name, int beats){
		// add move to sequence
		this.sequence.add(this.moves.get(name));

		// add beat time to sequence
		this.beats.add(beats);

		// return self for chaining
		return this;
	}

	/**
	 * @brief Consecutively loop a set of moves
	 *
	 * @param numberOfMoves amount of moves to loop
	 * @return self for chaining
	 *
	public DanceSequence loopLastMoves(int numberOfMoves){
		// get start index
		int startIndex = this.sequence.size() - numberOfMoves;
		int endIndex = this.sequence.size() - 1;

		return this.repeatFromStart(startIndex, endIndex);
	}

	public DanceSequence repeat(int start, int end) {
		// get indexes
		int startIndex = this.sequence.size() - start;
		int endIndex = this.sequence.size() - end;

		return this.repeatFromStart(startIndex, endIndex);
	}

	public DanceSequence repeatFromStart(int startIndex, int endIndex){
		for(int i = startIndex; i <= endIndex; i++){
			// re-add move
			this.sequence.add(this.sequence.get(i));

			// re-add beat
			this.beats.add(this.beats.get(i));
		}

		return this;
	}

	public DanceSequence loop(){
		return this.repeatFromStart(0, this.sequence.size()-1);
	}

	/**
	 * @brief clears all prior moves.  mainly for debugging purposes since it's easier than commenting the lines
	 *
	 * @return self for chaining
	 *
	public DanceSequence clearPrior(){
		this.sequence.clear();

		this.beats.clear();

		return this;
	}

	public void dance(AbeBot bot, LinearOpMode opMode, boolean loop){
		// current move index
		int moveIndex = 0;

		// restart beat keeper
		this.beatKeeper.restart();

		// schedule first move
		this.beatKeeper.scheduleBeatIn(0);

		while(opMode.opModeIsActive()){
			if(this.beatKeeper.isFiring()){
				// do moveIndex move
				this.sequence.get(moveIndex).doMove(bot);

				// schedule next move
				this.beatKeeper.scheduleBeatIn(this.beats.get(moveIndex));

				// increment moveIndex
				moveIndex++;

				// check if finished
				if(moveIndex >= this.sequence.size()){
					// if looping, reset to start
					if(loop){
						moveIndex = 0;
					}
					// otherwise, break loop
					else {
						break;
					}
				}
			}

			if(this.sequence.get(moveIndex-1).getState(DanceMoveBitField.DRIVE_TRANSLATION_BYTES) != DanceMoveBitField.DRIVE_TRANSLATION_NONE) {
				// do moveIndex move again
				this.sequence.get(moveIndex-1).doMove(bot);
			}

			bot.drive.update();
		}
	}
	*/
}
