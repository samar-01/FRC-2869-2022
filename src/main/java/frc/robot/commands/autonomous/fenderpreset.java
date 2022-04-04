// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.*;
import static frc.robot.Constants.*;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class fenderpreset extends SequentialCommandGroup {
	/** Creates a new fenderfindleft. */
	public fenderpreset() {
		// Add your commands in the addCommands() call, e.g.
		// addCommands(new FooCommand(), new BarCommand());
		double angle = 35;
		double dist = 1.2;
		// addCommands(new fenderhigh(), new RotateNoEncoder(false, angle), new PointIntake(), new DriveIntake(dist), new DriveIntake(-dist), new RotateNoEncoder(true, angle), new DriveDistance(autodistance));
		addCommands(new fenderhigh(), new RotateNoEncoder(false, angle), new DriveIntake(dist), new DriveIntake(-dist), new RotateNoEncoder(true, angle+10), new DriveDistance(autodistance+0.6, 0.75), new FenderShot());
	}
}