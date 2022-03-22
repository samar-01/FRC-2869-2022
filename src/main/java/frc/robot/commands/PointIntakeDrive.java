// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.RobotContainer;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import static frc.robot.Constants.*;

public class PointIntakeDrive extends CommandBase {
	DrivetrainSubSys drive;
	ShooterSubSys shooterSubSys;
	/** Creates a new PointIntake. */
	public PointIntakeDrive(DrivetrainSubSys drive, ShooterSubSys shooterSubSys) {
		this.drive = drive;
		this.shooterSubSys = shooterSubSys;
		addRequirements(drive, shooterSubSys);
		// addRequirements(drive);
		// Use addRequirements() here to declare subsystem dependencies.
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		drive.resetPID();
		shooterSubSys.intake();
		status.setString("point driving");
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		drive.ballTurnDrive();
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		drive.stop();
		drive.resetPID();
	}
	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		// return drive.isBallPointed();
		return !shooterSubSys.isIntakeEmpty();
		// return false;
	}
}
