// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.angle;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.AngleSubSys;

public class Angle extends CommandBase {

	private final AngleSubSys angleSubSys;

	/** Creates a new Angle. */
	public Angle(AngleSubSys angleSubSys) {
		this.angleSubSys = angleSubSys;
		addRequirements(angleSubSys);
		// Use addRequirements() here to declare subsystem dependencies.
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		AngleSubSys.init();
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		angleSubSys.run();
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
