// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubSys;

public class PreRev extends CommandBase {
	ShooterSubSys shooterSubSys;
	/** Creates a new PreRev. */
	public PreRev(ShooterSubSys shooterSubSys) {
		this.shooterSubSys = shooterSubSys;
		addRequirements(shooterSubSys);
		// Use addRequirements() here to declare subsystem dependencies.
	}

	boolean done = false;
	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		System.out.println("prerev");
		shooterSubSys.init();
		shooterSubSys.rev();
		done = true;
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return done;
	}
}
