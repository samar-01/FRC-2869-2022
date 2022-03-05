// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.opencv.core.Point;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.AngleSubSys;
import frc.robot.subsystems.DrivetrainSubSys;
import frc.robot.subsystems.LimelightSubSys;
import frc.robot.subsystems.ShooterSubSys;
// import frc.robot.commands.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoShoot extends ParallelCommandGroup {

	/** Creates a new AutoShoot. */
	public AutoShoot(ShooterSubSys shooterSubSys, DrivetrainSubSys drivetrainSubSys, AngleSubSys angleSubSys, LimelightSubSys limelightSubSys) {
		// Add your commands in the addCommands() call, e.g.
		// addCommands(new FooCommand(), new BarCommand());
		addCommands(new AutoLift(angleSubSys), new AutoPoint(drivetrainSubSys, angleSubSys, limelightSubSys), new AutoShootSpeed(shooterSubSys, angleSubSys, drivetrainSubSys));
	}
}
