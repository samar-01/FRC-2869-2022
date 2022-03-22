// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubSys;

public class Rotate extends CommandBase {
	DrivetrainSubSys drive;
	double angle;
	/** Creates a new Drivetrain. */
	public Rotate(double angle) {
		this.drive = RobotContainer.drivetrainSubSys;
		this.angle = angle;
		addRequirements(drive);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		drive.setRot(angle);
		drive.resetPID();
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		drive.spin();
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		// drive.stop();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		if (drive.spun) {
			drive.spun = false;
			return true;
		}
		return false;
	}
}
