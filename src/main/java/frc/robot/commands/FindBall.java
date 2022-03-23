// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubSys;
import static frc.robot.Constants.*;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

public class FindBall extends CommandBase {
	DrivetrainSubSys drivetrainSubSys;
	double tolerance;

	private static final AHRS ahrs = new AHRS(SPI.Port.kMXP);
	private boolean turnRight = true;

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
		ahrs.reset();
	}

	boolean done = false;

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if (pTrack() != Double.POSITIVE_INFINITY) {
			done = true;
		} else if (ahrs.getYaw() > tolerance) {
			turnRight = false;
		} else if (ahrs.getYaw() < -tolerance) {
			turnRight = true;
		}
		if (turnRight) {
			drivetrainSubSys.drv(0, 0.3);
		} else {
			drivetrainSubSys.drv(0, -0.3);
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return done;
	}
}
