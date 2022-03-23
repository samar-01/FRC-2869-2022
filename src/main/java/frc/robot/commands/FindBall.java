// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubSys;
import static frc.robot.Constants.*;

public class FindBall extends CommandBase {
	DrivetrainSubSys drivetrainSubSys;
	double tolerance;
	/** Creates a new FindBall. */
	public FindBall(double tolerance) {
		this.tolerance = tolerance;
		this.drivetrainSubSys = RobotContainer.drivetrainSubSys;
		addRequirements(drivetrainSubSys);
		// Use addRequirements() here to declare subsystem dependencies.
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		onFlash();
		status.setString("finding ball");
	}

	boolean done = false;
	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if (pTrack() != Double.POSITIVE_INFINITY){
			done = true;
		} else {
			drivetrainSubSys.drv(0, 0.2); // TODO check if too fast/slow
		}
		// TODO yell at ankur if he doesnt implement navx stuff to avoid turning too far
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return done;
	}
}
