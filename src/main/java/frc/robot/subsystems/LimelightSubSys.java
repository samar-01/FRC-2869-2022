// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubSys extends SubsystemBase {
	/** Creates a new LimelightSubSys. */
	public LimelightSubSys() {
	}

	public static double getLimeDouble(String str) {
		return getLimeDouble(str, 0.0);
	}

	public static double getLimeDouble(String str, double def) {
		return NetworkTableInstance.getDefault().getTable("limelight").getEntry(str).getDouble(def);
	}

	public static double getLimeX() {
		return getLimeDouble("tx");
	}

	public static double getLimeA() {
		return getLimeDouble("ta");
	}

	public static double getDistance() {
		// double horiz =
		// NetworkTableInstance.getDefault().getTable("limelight").getEntry("thor").getDouble(0);
		// // System.out.println(Arrays.toString(corners));
		// double distance = 221.27/horiz;
		// distance = Math.sqrt(Math.pow(distance,2) - 2.64);
		// SmartDashboard.putNumber("distance", distance);
		// SmartDashboard.putNumber("horiz", horiz);
		// // SmartDashboard.putNumber("distancein", distance/0.0254);
		// return distance;

		NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
		NetworkTableEntry ty = table.getEntry("ty");
		double targetOffsetAngle_Vertical = ty.getDouble(0.0);

		// how many degrees back is your limelight rotated from perfectly vertical?
		double limelightMountAngleDegrees = 28.5;

		// distance from the center of the Limelight lens to the floor
		double limelightLensHeightInches = 0.165;

		// distance from the target to the floor
		double goalHeightInches = 2.63;

		double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
		double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

		// calculate distance
		double dist = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
		dist += 2.0/3;
		SmartDashboard.putNumber("distance", dist);
		return dist;
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
