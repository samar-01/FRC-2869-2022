// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubSys;

public class DriveDistance extends CommandBase {
	
	private final DrivetrainSubSys drive;
  private final double distance;

	/** Creates a new Drivetrain. */
	public DriveDistance(DrivetrainSubSys drive, double distance) {
		this.drive = drive;
    this.distance = distance;
		addRequirements(drive);
	}
	
	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
    drive.setDrivePID(distance);
		// drive.resetEncoders();
	}
	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		drive.drivePID();
		// drive.autoDrive();
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		drive.stop();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return drive.drove;
	}
}
