
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;

public class drivetrainSubsystem extends SubsystemBase {

    private final PWMVictorSPX leftFront = new PWMVictorSPX(Constants.roboRIOPWMPorts.driveLeft1);
    private final PWMVictorSPX leftMiddle = new PWMVictorSPX(Constants.roboRIOPWMPorts.driveLeft2);
    private final PWMVictorSPX rightFront = new PWMVictorSPX(Constants.roboRIOPWMPorts.driveRight1);
    private final PWMVictorSPX rightMiddle = new PWMVictorSPX(Constants.roboRIOPWMPorts.driveRight2);

    private final SpeedControllerGroup left = new SpeedControllerGroup(leftFront, leftMiddle);
    private final SpeedControllerGroup right = new SpeedControllerGroup(rightFront, rightMiddle);
    private final DifferentialDrive drive = new DifferentialDrive(left, right);

    private final Joystick joystick = new Joystick(Constants.joystick.joystickPort);
    private final JoystickButton buttonX = new JoystickButton(joystick, 0);
    private final JoystickButton buttonA = new JoystickButton(joystick, 1);
    private final JoystickButton buttonB = new JoystickButton(joystick, 2);
    private final JoystickButton buttonY = new JoystickButton(joystick, 3);
    private final JoystickButton buttonLeft = new JoystickButton(joystick, 4);
    private final JoystickButton buttonRight = new JoystickButton(joystick, 5);
    private final JoystickButton buttonTopLeft = new JoystickButton(joystick, 6);
    private final JoystickButton buttonBack = new JoystickButton(joystick, 8);
    private final JoystickButton buttonStart = new JoystickButton(joystick, 9);
    private final JoystickButton buttonBigLeft = new JoystickButton(joystick, 10);
    private final JoystickButton buttonBigRight = new JoystickButton(joystick, 11);

    private static drivetrainSubsystem INSTANCE = new drivetrainSubsystem();

    public static drivetrainSubsystem getInstance() {
        if (INSTANCE == null) {
            synchronized (drivetrainSubsystem.class) {
                if (INSTANCE == null) {
                    INSTANCE = new drivetrainSubsystem();
                }
            }
        }
        return INSTANCE;
    }

    public void drive(double leftSpeed, double rightSpeed) {
        drive.tankDrive(leftSpeed, rightSpeed);
    }

    /** Creates a new drivetrainSubsystem. */

    public void setMotors(double leftSpeed, double rightSpeed) {
        this.left.set(leftSpeed / 12);
        this.right.set(rightSpeed / 12);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}