// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubSys;
import static frc.robot.Constants.*;

import org.opencv.core.Mat;
public class GoToPose extends CommandBase {
	DrivetrainSubSys drive;
	Pose2d tarpos;

	boolean rotated = false;

	/** Creates a new gotoPos. */
	public GoToPose(Pose2d tarpos) {
		drive = RobotContainer.drivetrainSubSys;
		this.tarpos = tarpos;
		addRequirements(drive);
		// Use addRequirements() here to declare subsystem dependencies.
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {

	}

	boolean done = false;

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if (!rotated){
			drive.drv(0, clamp((tarpos.getRotation().getDegrees() - drive.getGyroHeading())/20, autoturnspeed, autoturnspeed));
			if (Math.abs(tarpos.getRotation().getDegrees() - drive.getGyroHeading()) < 3){
				rotated = true;
			}
		} else {

		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		drive.stop();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return done;
	}
}
