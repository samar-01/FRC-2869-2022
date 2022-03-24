// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer;
import frc.robot.commands.*;
import frc.robot.commands.autonomous.*;
import frc.robot.subsystems.*;

import static frc.robot.Constants.*;

import java.util.Map;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
	private Command m_autonomousCommand;
	private RobotContainer m_robotContainer;
	double autorotate = 120;
	ShuffleboardTab auto = Shuffleboard.getTab("Auto");
	ShuffleboardTab teleop = Shuffleboard.getTab("Teleop");

	/**
	 * This function is run when the robot is first started up and should be used for any
	 * initialization code.
	 */
	@Override
	public void robotInit() {
		System.out.println("----ROBOT INIT----");
		// Instantiate our RobotContainer.  This will perform all our button bindings, and put our
		// autonomous chooser on the dashboard.
		m_robotContainer = new RobotContainer();
		initFlash();
		AngleSubSys.init();
		RobotContainer.climberSubSys.init();
		networkInit();
		offLime();

		ShuffleboardTab auto = Shuffleboard.getTab("Auto");

		// Command driveBack = new driveBack();
		// Command driveShoot = new driveShoot();
		// Command twoBallRight = new twoBallLeft();
		// Command ballfind = new ballfind();
		// Command testrot = new testrot();
		// Command none = new none();
		
		// autopicker.addOption("driveBack", driveBack);
		// autopicker.setDefaultOption("driveShoot", driveShoot);
		// autopicker.addOption("2 ball right", twoBallRight);
		// autopicker.addOption("ballfind", ballfind);
		// autopicker.addOption("testrot", testrot);
		// autopicker.addOption("none", none);
		
		// autoPickerEntry = auto.add("autoselector", driveBack).withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(0, 0).withSize(3, 1).getEntry();
		// auto.add("auto", autopicker).withPosition(0, 0).withSize(3, 1);

		newautopick.addOption("back", automodes.backupOnly);
		newautopick.setDefaultOption("fender", automodes.fender);
		newautopick.addOption("backshoot", automodes.backupShoot);
		newautopick.addOption("leftball", automodes.left2ball);
		newautopick.addOption("none", automodes.none);
		newautopick.addOption("ballfind", automodes.ballfind);
		newautopick.addOption("testrot", automodes.testrot);

		auto.add("auto", newautopick).withPosition(0, 0).withSize(3, 1);

		// autorotateEntry = auto.add("autorotate", autorotate).withPosition(0, 0).withSize(1, 1).getEntry();
		angleEntryA = auto.add("Angle", -34).withPosition(0, 1).withSize(1, 1).getEntry();
		batVoltageEntryA = auto.add("Voltage", 0).withPosition(1, 1).withSize(1, 1).getEntry();
		timeA = auto.add("time", 0).withPosition(2, 1).getEntry();
		statusA = auto.add("status", "init").withPosition(0, 2).withSize(3, 1).getEntry();
		
		// autopicker.setDefaultOption("backup shoot", automodes.backupShoot);
		// autopicker.addOption("backup only", automodes.backupOnly);
		// autopicker.addOption("2 ball right", automodes.backupShootTurnRight);
		// autoPickerEntry = auto.add("autoselector", autopicker).withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(0, 0).withSize(3, 1).getEntry();
		// auto.add("autoselector", 0);
		
		ShuffleboardTab teleop = Shuffleboard.getTab("Teleop");
		distanceEntry = teleop.add("Distance", 0.0).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 1, "max", 7)).withPosition(0, 0).withSize(3, 1).getEntry();
		velconstantEntry = teleop.addPersistent("velconstant", 6.78).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 6, "max", 10)).withPosition(0,1).withSize(3, 1).getEntry();
		launchvelconstantEntry = teleop.addPersistent("launchvelconstant", 1).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0.5, "max", 1.5)).withPosition(0,2).withSize(3, 1).getEntry();
		slider775 = teleop.addPersistent("slider775", 1).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 5, "max", 12)).withPosition(0,3).withSize(3, 1).getEntry();
		ballinEntry = teleop.add("BallIn", false).withPosition(3, 0).withSize(1, 1).getEntry();
		revedEntry = teleop.add("Reved", false).withPosition(4, 0).withSize(1, 1).getEntry();
		angleEntryT = teleop.add("Angle", -34).withPosition(5, 0).withSize(1, 1).getEntry();
		// leftFalEntry = teleop.add("lfalc", 0).withPosition(0, 4).withSize(1, 1).getEntry();
		// rightFalEntry = teleop.add("rfalc", 0).withPosition(1, 4).withSize(1, 1).getEntry();
		tarSpeedEntry = teleop.add("target", 0).withPosition(1, 4).withSize(1, 1).getEntry();
		batVoltageEntryT = teleop.add("battery",0).withPosition(1, 4).withSize(1, 1).getEntry();
		timeT = teleop.add("time", 0).withPosition(0, 4).withSize(1, 1).getEntry();
		statusT = teleop.add("status", "init").withPosition(2, 4).withSize(3, 1).getEntry();
		// tarspeedthing = teleop.add("tarspeedthing", 7000).withPosition(4, 4).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 7000, "max", 9000)).withSize(3, 1).getEntry();
		Shuffleboard.selectTab("Auto");

		batVoltageEntry = batVoltageEntryA;
		angleEntry = angleEntryA;
		time = timeA;
		status = statusA;
	}

	/**
	 * This function is called every robot packet, no matter the mode. Use this for items like
	 * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
	 *
	 * <p>This runs after the mode specific periodic functions, but before LiveWindow and
	 * SmartDashboard integrated updating.
	 */
	@Override
	public void robotPeriodic() {
		// Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
		// commands, running already-scheduled commands, removing finished or interrupted commands,
		// and running subsystem periodic() methods.  This must be called from the robot's periodic
		// block in order for anything in the Command-based framework to work.
		CommandScheduler.getInstance().run();
		batVoltageEntry.setNumber(RobotController.getBatteryVoltage());
		time.setNumber(RobotContainer.shooterSubSys.calcVel());
		// time.setNumber(Timer.getMatchTime());
	}

	/** This function is called once each time the robot enters Disabled mode. */
	@Override
	public void disabledInit() {
		DrivetrainSubSys.stopDrive();
		offFlash();
		networkInit();
		offLime();
	}

	@Override
	public void disabledPeriodic() {

	}

	// void addData(String tab, String label, Sendable sendable){
	// 	try {
	// 		Shuffleboard.getTab(tab).add(label, sendable);
	// 	} catch (IllegalArgumentException e){
			
	// 	}
	// }

	// Timer autodrive = new Timer();
	/** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
	@Override
	public void autonomousInit() {
		batVoltageEntry = batVoltageEntryA;
		angleEntry = angleEntryA; // TODO check that angle doesnt go to low in auto
		time = timeA;
		status = statusA;
		photonResetPipe();
		Shuffleboard.selectTab("Auto");
		autorotate = SmartDashboard.getNumber("autorotate", autorotate);
		RobotContainer.drivetrainSubSys.autoInit();

		m_autonomousCommand = new Autonomous(newautopick.getSelected());
		// m_autonomousCommand = autopicker.getSelected();
		
		initLime();
		offFlash();
		
		// schedule the autonomous command (example)
		if (m_autonomousCommand != null) {
			System.out.println("auto scheduled");
			m_autonomousCommand.schedule();
		}
		// autodrive.reset();
		// autodrive.start();
		// AngleSubSys.init();
	}
	double backup = 5;
	/** This function is called periodically during autonomous. */
	@Override
	public void autonomousPeriodic() {
		// autoSchedule(RobotContainer.autonomous);
		// if (!autodrive.hasElapsed(backup)){
			// 	DrivetrainSubSys.drv(-0.40, 0);
			// } else {
		// 	DrivetrainSubSys.stopDrive();
		// }
		
		// if (autodrive.hasElapsed(backup+0.2)){
		// 	AngleSubSys.lift();
		// }
		// if (autodrive.hasElapsed(backup+0.2) && AngleSubSys.isLifted()){
		// 	ShooterSubSys.revup();
		// }
	}

	@Override
	public void teleopInit() {
		batVoltageEntry = batVoltageEntryT;
		angleEntry = angleEntryT;
		time = timeT;
		status = statusT;
		Shuffleboard.selectTab("Teleop");
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}
		initLime();

		/* photon vision temp comment
		autoDriveButton.whenPressed(new DriveAuto(new DrivetrainSubSys()));
		resetDriveButton.whenPressed(new DriveReset(new DrivetrainSubSys()));
		stopDriveButton.whenPressed(new DriveStop(new DrivetrainSubSys()));
		
		driveDriveButton.whenPressed(new DriveDistance(new DrivetrainSubSys(),20));
		*/
		
		// spinDriveButton.whenPressed(new Drive180(new DrivetrainSubSys()));
		
		offFlash();
		// onFlash();
		onLime();
		// pointDriveButton.whenPressed(new AutoPoint(new DrivetrainSubSys()));
		// closeHigh.whenPressed(new CloseHigh());

		try{
			HttpCamera ll = new HttpCamera("limelight", "http://limelight.local:5800/stream.mjpeg"); // TODO check this
			teleop.add("limelight", ll).withPosition(3, 0).withSize(3, 3);
		} catch (Exception e){

		}
	}

	public void initLime(){
		networkInit();
		NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
	}
	
	
	/** This function is called periodically during operator control. */
	@Override
	public void teleopPeriodic() {
		// System.out.println(getFlashCurrent());
		// System.out.println(RobotContainer.shooterSubSys.left775.getSupplyCurrent());
		// status.setDouble(getFlashCurrent());
		
		// System.out.println(velconstantEntry.getNumber(0));
		// System.out.println(ShooterSubSys.isIntakeEmpty());

		if (opxbox.getRightStickButtonPressed()){
			if (getFlash() > 0){
				offFlash();
			} else {
				onFlash();
			}
		}

		if (!RobotContainer.pointIntake.isScheduled()){
			if (xbox.getPOV() == 90 && !RobotContainer.pointIntake.isScheduled()){
				autoSchedule(RobotContainer.pointIntake);
			} else {
				autoSchedule(RobotContainer.drivetrain);
				autoSchedule(RobotContainer.shooter);
			}
		} else {
			RobotContainer.drivetrain.cancel();
			RobotContainer.shooter.cancel();
		}

		if (xbox.getPOV() == 270){
			RobotContainer.pointIntake.cancel();
		}

		autoSchedule(RobotContainer.climber);
		if (opxbox.getAButton()){
			// RobotContainer.shooterSubSys.rev();
			autoSchedule(RobotContainer.autoLift);
		} else if (opxbox.getBButton()){
			// RobotContainer.shooterSubSys.rev();
			autoSchedule(RobotContainer.autoDown);
		} else if (opxbox.getYButton()){
			// RobotContainer.shooterSubSys.rev();
			autoSchedule(RobotContainer.autoMid);
		} else if (opxbox.getPOV() == 270){
			autoSchedule(RobotContainer.closeHigh);
		} else if (!RobotContainer.pointIntake.isScheduled() && !RobotContainer.autoLift.isScheduled() && !RobotContainer.autoDown.isScheduled() && !RobotContainer.autoMid.isScheduled()){
			autoSchedule(RobotContainer.shooter);
			autoSchedule(RobotContainer.angle);
		}
		
		// LimelightSubSys.getDistance();
		distanceEntry.setDouble(distance());
	}

	@Override
	public void testInit() {
		// Cancels all running commands at the start of test mode.
		CommandScheduler.getInstance().cancelAll();
	}

	/** This function is called periodically during test mode. */
	@Override
	public void testPeriodic() {}
}
