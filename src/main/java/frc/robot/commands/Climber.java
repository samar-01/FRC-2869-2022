// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubSys;

public class Climber extends CommandBase {
	ClimberSubSys climberSubSys;
	/** Creates a new Climber. */
	public Climber(ClimberSubSys climberSubSys) {
		this.climberSubSys = climberSubSys;
		addRequirements(climberSubSys);
		// Use addRequirements() here to declare subsystem dependencies.
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		climberSubSys.init();
	}
	
	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		climberSubSys.run();
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
