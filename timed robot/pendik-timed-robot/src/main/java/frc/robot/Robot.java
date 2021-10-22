/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import javax.swing.Timer;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * The VM is configured to automatically run this class, and to call the methods
 * corresponding to each mode, as described in the TimedRobot documentation. If
 * you change the name of this class or the package after creating this project,
 * you must also update the build.gradle file in the project.
 */

public class Robot extends TimedRobot {

  public PWMVictorSPX left = new PWMVictorSPX(1);
  public PWMVictorSPX right = new PWMVictorSPX(2);

  public DifferentialDrive myDrive = new DifferentialDrive(left, right);

  public Timer theTimer = new Timer();

  public Joystick myJoystick = new Joystick(0);


  /**
   * This method is run when the robot is first started up and should be used for
   * any initialization code.
   */
  @Override
  public void robotInit() {

  }

  /**
   * This method is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   * <p>
   * This runs after the mode specific periodic methods, but before LiveWindow and
   * SmartDashboard integrated updating.
   */

  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString line to get the
   * auto name from the text box below the Gyro
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure below with additional strings. If using the SendableChooser
   * make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    theTimer.reset();
    theTimer.start();
  }

  /**
   * This method is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    if (theTimer.get() < 4.0) {
      myDrive.tankDrive(1, 1);
    } else {
      myDrive.stopMotor();
      theTimer.stop();
    }
  }

  /**
   * This method is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    myDrive.tankDrive(myJoystick.getRawAxis(1), myJoystick.getRawAxis(3));

    leftMC.set(myJoystick.getRawAxis(1));
    rightMC.set(myJoystick.getRawAxis(3));

  }

  /**
   * This method is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}