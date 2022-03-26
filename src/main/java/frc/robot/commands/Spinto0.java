// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubSys;

public class Spinto0 extends CommandBase {
	DrivetrainSubSys drive;
	/** Creates a new Spinto0. */
	public Spinto0() {
		// Use addRequirements() here to declare subsystem dependencies.
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		this.drive = RobotContainer.drivetrainSubSys;
		addRequirements(drive);
		drive.setRot0();
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		drive.spin();
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		drive.stop();
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
