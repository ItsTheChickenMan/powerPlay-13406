package org.firstinspires.ftc.teamcode.lib.dance;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * @brief class for keeping time to a bpm
 */
public class BeatKeeper {
	// timing info
	private double bpm;
	private double secondsPerBeat;

	// timer
	private ElapsedTime timer;

	// scheduler
	// can only schedule one beat at a time
	public int nextScheduledBeat;

	// defaults to true so that it doesn't fire until scheduled to
	private boolean beatFlushed = true;

	public BeatKeeper(double bpm){
		this.bpm = bpm;
		this.secondsPerBeat = 60.0 / this.bpm;
		this.timer = new ElapsedTime();
		this.nextScheduledBeat = 0;
	}

	/**
	 * @brief restarts timer
	 */
	public void restart(){
		// reset timer to make timing accurate
		this.timer.reset();
	}

	/**
	 * @return total number of beats that have passed since the last call to restart() (or since construction, if start was never called)
	 */
	public int totalBeats(){
		return (int)(this.timer.seconds() / this.secondsPerBeat);
	}

	/**
	 * @brief Have the class fire in a certain number of beats from now
	 *
	 * @param beats number of beats from now until next fire
	 */
	public void scheduleBeatIn(int beats){
		// reset flush state
		this.beatFlushed = false;

		// schedule next beat
		this.nextScheduledBeat = this.totalBeats() + beats;
	}

	/**
	 * @brief whether the beat provided has already happened, or is currently happening.
	 *
	 * @param beat
	 */
	public boolean isBeatPassed(int beat){
		return beat <= this.totalBeats();
	}

	/**
	 * @return if we're currently firing for the scheduled beat
	 */
	public boolean isFiring(){
		boolean out = this.isBeatPassed(this.nextScheduledBeat);

		if(out){
			if(!this.beatFlushed){
				this.beatFlushed = true;
			} else {
				return false;
			}

			return out;
		}

		return false;
	}
}