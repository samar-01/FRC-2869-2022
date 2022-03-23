// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// 100 cycles rotates 60 deg
// each cycle rotates (3/5) deg

import frc.robot.RobotContainer;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import static frc.robot.Constants.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class twoBallLeft extends SequentialCommandGroup {
	/** Creates a new twoBallRight. */
	public twoBallLeft() {
		// Add your commands in the addCommands() call, e.g.
		// addCommands(new FooCommand(), new BarCommand());
		double angle = 60;
		angle = (5*angle)/3;
		// addCommands(new driveBack(), new LiftAlignShoot(), new DownRotate(angle), new FindBall(), new PointIntakeDrive(), new Rotate(-angle));
		// addCommands(new driveBack(), new LiftShoot(), new AutoDown(), new RotateNoEncoder(false, (int)angle), new FindBall(), new RotateNoEncoder(true, (int)angle), new LiftAlignShoot(), new AutoDown());
		addCommands(new driveShoot(), new RotateNoEncoder(false, angle), new FindBall(30), new PointIntakeDrive(), new RotateNoEncoder(true, angle), new AutoLift(), new AutoFindGoal(), new AutoPointGoal(), new AutoShoot(), new AutoDown());
	}
}
