// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubSys;
import frc.robot.subsystems.ShooterSubSys;

import static frc.robot.Constants.*;

public class DriveIntake extends CommandBase {

	private final DrivetrainSubSys drive;
	double distance;
	ShooterSubSys shooterSubSys;

	public DriveIntake() {
		this.drive = RobotContainer.drivetrainSubSys;
		// this.distance = encToDist(distance);
		// this.distance = distance;
		this.shooterSubSys = RobotContainer.shooterSubSys;
		addRequirements(drive, shooterSubSys);
	}
	/**
	 * 
	 * @param distance meters
	 */
	public DriveIntake(double distance) {
		this.drive = RobotContainer.drivetrainSubSys;
		// this.distance = encToDist(distance);
		this.distance = distToEnc(distance);
		// this.distance = distance;
		this.shooterSubSys = RobotContainer.shooterSubSys;
		addRequirements(drive, shooterSubSys);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		// drive.setDrivePID(distance);
		drive.resetEncoders();
		status.setString("driving back");
		shooterSubSys.intake();
	}

	boolean done = false;
	double speed = 0.6, tolerance = 3;
	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		// drive.drivePID();
		// drive.autoDrive();
		if (Math.abs(drive.getEncDistance() - distance) < tolerance){
			done = true;
		}
		if (drive.getEncDistance() > distance){
			drive.drv(-speed, 0);
		} else if (drive.getEncDistance() < distance){
			drive.drv(speed, 0);
		} else {
			done = true;
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		drive.stop();
		shooterSubSys.stop();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return done;
	}
}
