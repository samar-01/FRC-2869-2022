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
 * lifts and revs
 */
public class AutoDown extends CommandBase {

	private final AngleSubSys angleSubSys;
	private final ShooterSubSys shooterSubSys;

	/** Creates a new Angle. */
	public AutoDown() {
		this.angleSubSys = RobotContainer.angleSubSys;
		this.shooterSubSys = RobotContainer.shooterSubSys;
		addRequirements(angleSubSys, shooterSubSys);
		// Use addRequirements() here to declare subsystem dependencies.
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		angleSubSys.init();
		angleSubSys.resetPID();
		shooterSubSys.stop();
		angleSubSys.setTargetLow();
		status.setString("angling down");
	}

//arsh is epic

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		angleSubSys.pidmove();
		// angleSubSys.lift();
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		if (angleSubSys.isLifted()){
			angleSubSys.stop();
		}
		return angleSubSys.isLifted();
	}
}
