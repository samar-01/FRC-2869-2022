// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.RobotContainer;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import static frc.robot.Constants.*;

public class Shoot775 extends CommandBase {
	/** Creates a new Shoot775. */
	ShooterSubSys shooterSubSys;
	public Shoot775() {
		this.shooterSubSys = RobotContainer.shooterSubSys;
		addRequirements(shooterSubSys);
		// Use addRequirements() here to declare subsystem dependencies.
	}

	Timer timer = new Timer();
	double shoottime = 0.5;

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		shooterSubSys.shoot();
		timer.start();
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		shooterSubSys.stop();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return timer.hasElapsed(shoottime);
	}
}
