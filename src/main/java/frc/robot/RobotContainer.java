// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import org.photonvision.PhotonCamera;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  public final DriveSubsystem m_robotDrive = new DriveSubsystem();

  // The driver's controller
  XboxController m_driveController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_clawController = new XboxController(OIConstants.kClawControllerPort);

  public final CANSparkMax m_shooterAccelerator = new CANSparkMax(12, MotorType.kBrushed);
  public final CANSparkMax m_shooterRight = new CANSparkMax(13, MotorType.kBrushless);
  public final CANSparkMax m_shooterLeft = new CANSparkMax(14, MotorType.kBrushless);

  public final CANSparkMax m_hang = new CANSparkMax(17, MotorType.kBrushed);
  public final CANSparkMax m_intake = new CANSparkMax(15, MotorType.kBrushed);
  public final CANSparkMax m_intakeContraRoller = new CANSparkMax(16, MotorType.kBrushed);

  PhotonCamera m_photonCamera = new PhotonCamera("PI Cam");

  Servo m_hangRelease = new Servo(1);

  int revPHId = 2;
  public final DoubleSolenoid m_armSolenoid = new DoubleSolenoid(revPHId, PneumaticsModuleType.REVPH, 0, 1);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_intake.setInverted(false);
    m_intakeContraRoller.setInverted(true);
    m_shooterLeft.setInverted(false);
    m_shooterRight.setInverted(true);
  }
}
