// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import static frc.robot.Constants.*;

public class AutomaticSubSys extends SubsystemBase {
	AngleSubSys angleSubSys;
	DrivetrainSubSys drivetrainSubSys;
	AutoPoint point;
	AutoLift lift;
	/** Creates a new Automatic. */
	public AutomaticSubSys(AngleSubSys angleSubSys, DrivetrainSubSys drivetrainSubSys, AutoPoint point, AutoLift lift) {
		this.angleSubSys = angleSubSys;
		this.drivetrainSubSys = drivetrainSubSys;
		this.point = point;
		this.lift = lift;
	}

	public void lift(){
		onLime();
		// angleSubSys.lift();
		lift.schedule();
		point.schedule();
	}

	public boolean lifted(){
		return false;
	}
	
	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
