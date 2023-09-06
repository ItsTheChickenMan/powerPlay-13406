# powerPlay-13406

Code for the robot "Abe", created by FTC 13406 "Consistently Inconsistent".

![Abraham running autonomously](/img/1+5auto.gif)

Here's Abe running autonomously in my living room, attempting a 1+5.  He misses one, but he tried his best.

## What is Abe?

Abraham is a robot created for the FTC 2022/2023 Season Game "PowerPlay".  His main mechanisms consist solely of a 4-wheel mecanum drivetrain and a robotic arm.  Counting the rotation of the drive train , Abe's arm has 4 degrees of freedom.

Abe's drivetrain consists of 4 mecanum wheels, which allow for holonomic movement, and 3 "dead" wheels -- unpowered wheels used for robot odometry.

Abe's arm is made of a set of linear slides attached to a rotating mechanism powered by a worm gear, mounted to the drive train.  At the end of the linear slides is where the wrist and claw are mounted.  The claw, made of 3D printed parts and rubbery grips, clamps around cones while the servo-actuated wrist changes the claw's relative pitch.

In tandem, these two systems work to allow Abe to reach his claw anywhere within a 4ft radius around him. 

## Design Goals

Given the high value of junction ownership and circuits in PowerPlay, our primary goal when designing Abe was to have a robot that could deposit a cone to any arbitrary junction at a fairly high speed, as opposed to specializing in depositing to a single high value junction at a higher speed.  From there, we were faced with a difficult challenge: designing a highly versatile robot that's also easy enough to control to make the versatility worthwhile.  We eventually settled on something vaguely similar to the present design: linear extension mounted to a rotating component, creating 2 degrees of freedom, and relying on drivetrain rotation to obtain a third degree of freedom.  This idea went through quite a few changes evolutions before settling on the design that Abe had for competition, but our goal of high versatility was realized.

## Build

As Lead Programmer, it is hard for me to do a comprehensive write-up on the build process.  However, I will note that we modelled and 3D printed quite a few important parts, including:

- Vertical REV Hub Mount
- Odometry Wheels Mounts
- Camera Mount
- Claw Parts

## Programming

### Driver-Controlled

We had guessed very early on in the prototyping process that Abe would require advanced programming in order to be drivable.  As we later found out, simply mapping motor speeds to controller states was not sufficiently fast enough for us to score rapidly and take full advantage of Abe's versatility.  Instead, we relied on inverse kinematics to determine the states needed to bring Abe's claw to anywhere in 3D space it could reach.  In addition to this, we programmed an "auto-aim" system for driver-controlled opmodes.  Using only the robot's current position and the input of a single joystick, the system will determine which nearby junction the driver is attempting to aim at with the direction of the joystick.  Then, it will determine the position of the junction and use the implemented inverse kinematics to determine the states needed to bring the claw to the top of the junction and deposit a cone.  This system reduced the complicated action of manually manipulating the position of the claw to the simple steps:

1. Aim joystick in the direction of the desired junction relative to the robot (the robot will rotate towards the junction and rotate the arm to aim at it)
2. Hold the extension trigger down to extend the arm to the top of the junction
3. Hold the wrist bumper to rotate the wrist downward, planting the grasped cone over the top of the junction and acting as a safety check to ensure that the cone will deposit
4. Press the deposit button to drop the cone and reset the arm

Allowing us to rapidly deposit to any junction fairly easily, taking full advantage of Abe's versatile arm.

### Auto

We also use the auto-aim system during the Autonomous period.  This allows Abe to adapt to slight changes in position that tend to occur due to rapid rotation or general interference, making the auto considerably more consistent.  In addition, we take advantage of a Finite State Machine with 7 total states, which made autonomous programs considerably easier to write and modify for varying behaviors.

## Regrets/End of Season Conclusion

Abe did really well overall: he objectively has the best auto of any 13406 robot ever made, and is the only 13406 robot to be on the winning alliance at the Massachusetts State Championship.  However, I do have a few regrets:

- We should not have relied on the drivetrain for a degree of freedom for the arm.  We found out during the season that reliably aiming the drivetrain towards a point is ridiculously difficult to accomplish, and not particularly fast, either.  Having the arm be mounted to a turntable or similar would likely have been faster and more accurate overall.  Additionally, we found that rotating as frequently and violently as Abe does, coupled with the weight distribution freqently shifting, messes with odometry quite a bit.  We were unable to ever get odometry to a point where it was reasonably accurate with frequent and rapid rotation, and required a few hacky tricks in auto in order to have any reliability.  Once again, this would have been solved with having the arm mounted on a turntable or similar and not having the bot rotate at all, if possible.
- We should not have waited until just a few weeks before our first qualifier to start implementing odometry.  This one's pretty straightforward: we waited too long and didn't have odometry working until days before our qualifier #1.  This left practically no time to practice driving, leaving in a lot of practical issues that would have been noticed with practice as well as resulting in a few driver errors.  Additionally, we had no cone-scoring auto as there was no time to implement it.  We were quite lucky that we had more time before the second qualifier to work out the issues and implement a scoring auto.
- More work could have been put in at the start of the season to keep code clean, so that messy code didn't pile up and get forgotten when the late season rush kicked in.  A lot of time was wasted in the late season on silly code issues that were mostly the result of rushed documentation (or none at all) and general implementation laziness.


Overall, this was definitely a fantastic season.  It certainly wasn't easy; there were quite a few long nights of troubleshooting for everyone, but it was quite worth it in the end.  I'm happy to be able to graduate robotics on a good note and I'm eager to see where 13406 will go from here.
