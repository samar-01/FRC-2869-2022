package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

/**
 * used to get inputs (ie getIntake() or getShoot())
 */
public class Inputs {
	private static final XboxController xbox = new XboxController(0);
	private static final XboxController opxbox = new XboxController(1);
	private static final Joystick xboxjoystick = new Joystick(0);
	private static final Joystick opxboxjoystick = new Joystick(1);

	public static boolean getIntake(){
		return xbox.getAButton() || opxbox.getXButton();
	}

	public static boolean getRev(){
		return xbox.getXButton();
	}

	public static boolean getShoot(){
		return xbox.getBButton();
	}

	public static double getSpeed(){
		double speed = xbox.getRightTriggerAxis() - xbox.getLeftTriggerAxis();
		if (Math.abs(speed) < 0.1){
			speed = 0;
			// System.out.println("too slow");
		}
		return speed;
	}

	public static boolean getSlowSpeed(){
		return xbox.getRightBumper();
	}

	public static boolean getMidSpeed(){
		return xbox.getLeftBumper();
	}

	public static double getTurn(){
		return xbox.getLeftX();
	}

	public static boolean pidClimbUp(){
		return xbox.getPOV() == 0;
	}
	public static boolean pidClimbDown(){
		return xbox.getPOV() == 180;
	}
	public static boolean climbUp(){
		return opxbox.getPOV() == 0;
	}
	public static boolean climbDown(){
		return opxbox.getPOV() == 180;
	}

	public static boolean getFlashButton(){
		return opxbox.getRightStickButton();
	}

	public static boolean getFlashButtonPressed(){
		return opxbox.getRightStickButtonPressed();
	}

	public static boolean getStartBallTrack(){
		return xbox.getPOV() == 90;
	}

	public static boolean getCancelBallTrack(){
		return xbox.getPOV() == 270;
	}

	public static boolean getHigh(){
		return opxbox.getAButton();
	}
	
	public static boolean getLow(){
		return opxbox.getBButton();
	}

	public static boolean getMid(){
		return opxbox.getYButton();
	}

	public static boolean getFender(){
		return opxbox.getPOV() == 270;
	}

	public static boolean getFenderPressed(){
		return opxbox.getLeftBumperPressed();
	}

	public static boolean getLaunchpad(){
		return opxbox.getPOV() == 90;
	}

	public static boolean getOpOverride(){
		return opxbox.getStartButton();
	}

	public static boolean getSaveOverride(){
		return xbox.getStartButtonReleased();
	}

	public static double getArmPow(){
		return opxbox.getRightTriggerAxis() - opxbox.getLeftTriggerAxis();
	}

	@Deprecated
	public static boolean getEject(){
		// return opxbox.getLeftBumper();
		return false;
	}

	@Deprecated
	public static boolean getForceIn(){
		// return opxbox.getRightBumper();
		return false;
	}

	public static double getManualShooterSpeed(){
		return xbox.getRightY();
	}

}
