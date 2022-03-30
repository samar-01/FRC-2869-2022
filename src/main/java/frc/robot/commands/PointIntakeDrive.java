// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.RobotContainer;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import static frc.robot.Constants.*;
/**
 * moves towards ball until intaked
 */
public class PointIntakeDrive extends CommandBase {
	DrivetrainSubSys drive;
	ShooterSubSys shooterSubSys;
	/** Creates a new PointIntake. */
	public PointIntakeDrive() {
		this.drive = RobotContainer.drivetrainSubSys;
		this.shooterSubSys = RobotContainer.shooterSubSys;
		addRequirements(drive, shooterSubSys);
		// addRequirements(drive);
		// Use addRequirements() here to declare subsystem dependencies.
	}

	public PointIntakeDrive(double s) {
		this.drive = RobotContainer.drivetrainSubSys;
		this.shooterSubSys = RobotContainer.shooterSubSys;
		this.speed = s;
		addRequirements(drive, shooterSubSys);
		// addRequirements(drive);
		// Use addRequirements() here to declare subsystem dependencies.
	}


	double leftcount, rightcount;
	double leftencstart, rightencstart;
	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		// onFlash();
		drive.resetPID();
		shooterSubSys.intake();
		photonResetPipe();
		status.setString("point driving");
		c = 0;
		done = false;
		double encs[] = drive.getenc();
		leftencstart = encs[0];
		rightencstart = encs[1];
	}
	int c = 0;
	boolean done = false;
	double speed = 0.5;
	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		drive.ballTurnDrive();
		// c++;
		// if (!shooterSubSys.isIntakeEmpty() ||  c > 30 && shooterSubSys.get775Current() > 25){
		if (!shooterSubSys.isIntakeEmpty()){// ||  c > 30 && shooterSubSys.get775Current() > 25){
			done = true;
		}
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
		double encs[] = drive.getenc();
		leftcount = encs[0];
		rightcount = encs[1];
		distancedriven[0] = leftcount - leftencstart;
		distancedriven[1] = rightcount - rightencstart;
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
