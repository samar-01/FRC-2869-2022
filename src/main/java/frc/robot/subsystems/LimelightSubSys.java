// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubSys extends SubsystemBase {
	/** Creates a new LimelightSubSys. */
	public LimelightSubSys() {}

	
	public static double getLimeX(){
		return(NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0.0));
	}
	public static double getLimeA(){
		return(NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0.0));
	}

	
	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
