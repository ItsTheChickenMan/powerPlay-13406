package org.firstinspires.ftc.teamcode.lib.utils;

public class TaskThread {
	Actions actions;
	int delay;
	TaskRunnable taskRunnable;

	public TaskThread(Actions actions) {
		this(0, actions);
	}

	public TaskThread(int delay, Actions actions) {
		this.actions = actions;
		this.delay = delay;
		this.taskRunnable = new TaskRunnable();
	}

	/**
	 * An interface to be passed to a {@link TaskThread} constructor.
	 */
	public interface Actions {
		/**
		 * Robot code to be ran periodically on its own thread.
		 */
		public void loop();
	}

	public void start() {
		taskRunnable.start();
	}

	public void stop() {
		taskRunnable.stop();
	}

	class TaskRunnable implements Runnable {
		private Thread t;

		TaskRunnable() {

		}

		public void run() {
			try {
				while(!t.isInterrupted()) {
					actions.loop();
					Thread.sleep(delay);
				}
			} catch (InterruptedException e) {

			}
		}

		public void start() {
			if (t == null) {
				t = new Thread(this);
				t.start();
			}
		}

		public void stop() {
			t.interrupt();
		}
	}
}