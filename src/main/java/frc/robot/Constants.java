// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Joystick;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
	public static final XboxController xbox = new XboxController(0);
	public static final Joystick xboxjoystick = new Joystick(0);
	public static final JoystickButton autoDriveButton = new JoystickButton(xboxjoystick, 3);
	public static final JoystickButton resetDriveButton = new JoystickButton(xboxjoystick, 4);
	public static final JoystickButton spinDriveButton = new JoystickButton(xboxjoystick, 2);
	public static final AnalogInput ultra = new AnalogInput(2);
	public static double getUltra(){
		return (ultra.getVoltage()*1000)/9.77;
	}
	public static double clamp(double inp, double min, double max){
		if (inp > max){
			inp = max;
		}
		if (inp < min){
			inp = min;
		}
		return inp;
	}
}
