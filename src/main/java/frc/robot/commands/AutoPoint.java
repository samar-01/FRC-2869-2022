// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.AngleSubSys;
import frc.robot.subsystems.DrivetrainSubSys;
import frc.robot.subsystems.LimelightSubSys;

public class AutoPoint extends CommandBase {

	DrivetrainSubSys drive;
	AngleSubSys angleSubSys;
	LimelightSubSys limelightSubSys;
	/** Creates a new Drivetrain. */
	public AutoPoint(DrivetrainSubSys drivetrainSubSys, AngleSubSys angle, LimelightSubSys limelightSubSys) {
		this.drive = drivetrainSubSys;
		this.angleSubSys = angleSubSys;
		this.limelightSubSys = limelightSubSys;
		addRequirements(drivetrainSubSys, limelightSubSys, angleSubSys);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		drive.point(angleSubSys, limelightSubSys);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		// drive.stop();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return drive.finishPoint();
	}
}
