// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.RobotContainer;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import static frc.robot.Constants.*;

public class PointIntake extends CommandBase {
	DrivetrainSubSys drive;
	ShooterSubSys shooterSubSys;
	/** Creates a new PointIntake. */
	public PointIntake() {
		this.drive = RobotContainer.drivetrainSubSys;
		this.shooterSubSys = RobotContainer.shooterSubSys;
		addRequirements(drive, shooterSubSys);
		// addRequirements(drive);
		// Use addRequirements() here to declare subsystem dependencies.
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		// onFlash();
		drive.resetPID();
		shooterSubSys.intake();
		photonResetPipe();
		status.setString("point driving");
		done = false;
	}
	// int c = 0;
	boolean done = false;
	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		// drive.ballTurnDrive();
		drive.ballTurn();
		if (pTrack() != Double.POSITIVE_INFINITY && Math.abs(pTrack()) < 1){
			done = true;
		}
		// c++;
		// if (c > 30 && shooterSubSys.get775Current() > 25){
		// 	done = true;
		// }
		// System.out.println(shooterSubSys.get775Current());
		// drive.ballTurn();

	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		drive.stop();
		drive.resetPID();
		shooterSubSys.stop();
		offFlash();
	}
	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		// return drive.isBallPointed();
		return done;
		// return !shooterSubSys.isIntakeEmpty();
		// return false;
	}
}
