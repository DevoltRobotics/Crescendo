// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.time.Instant;
import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.auto.Auto2Notas;
import frc.robot.auto.Auto3NotasRojo;
import frc.robot.auto.Auto2NotasDeLado;
import frc.robot.auto.Auto3NotasAzul;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private RobotContainer m_robotContainer;

  private final String kNoAuto = "No Autonomo";
  private final String kAutoSoloPalFrente = "Solo Pal Frente";

  private final String kAutoUnaNota = "Una Nota";

  private final String kAuto3NotasRojo = "3 Notas Rojo";
  private final String kAuto3NotasAzul = "3 Notas Azul";

  private final String kAuto2NotasDeLado = "2 Notas de Lado";
  private final String kAuto2Notas = "2 Notas";

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    SmartDashboard.putStringArray("Auto List", new String[] { kNoAuto, kAutoSoloPalFrente, kAutoUnaNota, kAuto3NotasRojo, kAuto3NotasAzul, kAuto2NotasDeLado, kAuto2Notas });
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
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
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    switch(SmartDashboard.getString("Auto Selector", kNoAuto)) { 
      case kAutoSoloPalFrente:
        m_robotContainer.m_robotDrive.resetOdometry(new Pose2d());
        m_robotContainer.m_robotDrive.getGoToPointCommand(new Pose2d(2, 0, Rotation2d.fromDegrees(0)), 0.5).schedule();
        break;
      case kAuto3NotasRojo:
        Auto3NotasRojo.run(m_robotContainer);
        break;
      case kAuto3NotasAzul:
        Auto3NotasAzul.run(m_robotContainer);
        break;
      case kAuto2NotasDeLado:
        Auto2NotasDeLado.run(m_robotContainer);  
        break;
      case kAuto2Notas:
        Auto2Notas.run(m_robotContainer);
        break;
      case kAutoUnaNota:
        m_robotContainer.m_robotDrive.resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(180)));

        new InstantCommand(() -> {
          m_robotContainer.m_shooterLeft.set(1.0);
          m_robotContainer.m_shooterRight.set(1.0);
        }).andThen(
              m_robotContainer.m_robotDrive.getGoToPointCommand(new Pose2d(0.33, 0, Rotation2d.fromDegrees(180)), 1.0,
                      3))
              .andThen(
                      new InstantCommand(() -> m_robotContainer.m_armSolenoid.set(Value.kForward)))
              .andThen(
                      new WaitCommand(0.8))
              .andThen(
                      new InstantCommand(() -> {
                          m_robotContainer.m_shooterAccelerator.set(0.5);
                      }))
              .andThen(
                      new WaitCommand(0.5))
              .andThen(
                      new InstantCommand(() -> {
                          m_robotContainer.m_shooterAccelerator.set(0.0);
                          m_robotContainer.m_shooterLeft.set(0);
                          m_robotContainer.m_shooterRight.set(0);
                      })).schedule();
        break;
      default:
        break;
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  double teleopTurbo = 1.0;

  @Override
  public void teleopInit() {
    CommandScheduler.getInstance().cancelAll();

    m_robotContainer.m_photonCamera.setDriverMode(true);

    switch(DriverStation.getAlliance().orElseGet(() -> DriverStation.Alliance.Red)) {
      case Blue:
        m_robotContainer.m_robotDrive.resetOdometry(
          m_robotContainer.m_robotDrive.getPose().rotateBy(Rotation2d.fromDegrees(90))
        );
        break;
      case Red:  
        m_robotContainer.m_robotDrive.resetOdometry(
          m_robotContainer.m_robotDrive.getPose().rotateBy(Rotation2d.fromDegrees(-90))
        );
        break;
    }

    // Configure default commands
    m_robotContainer.m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> {
              var leftY = m_robotContainer.m_driveController.getLeftY();
              var leftX = m_robotContainer.m_driveController.getLeftX();
              var rightX = m_robotContainer.m_driveController.getRightX();

              if(Math.abs(leftY) <= 0.05) {
                leftY = 0;
              }
              if(Math.abs(leftX) <= 0.05) {
                leftX = 0;
              }
              if(Math.abs(rightX) <= 0.05) {
                rightX = 0;
              }

              m_robotContainer.m_robotDrive.drive(
                -MathUtil.applyDeadband(leftY * teleopTurbo, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(leftX * teleopTurbo, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(rightX * teleopTurbo, OIConstants.kDriveDeadband),
                true, true);
            },
            m_robotContainer.m_robotDrive));

    Trigger xButton = new Trigger(() -> m_robotContainer.m_clawController.getXButton());
    xButton.onTrue(new TogglingCommand(
      new InstantCommand(() -> m_robotContainer.m_armSolenoid.set(Value.kForward)),
      new InstantCommand(() -> m_robotContainer.m_armSolenoid.set(Value.kReverse))
    ));
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    if(m_robotContainer.m_driveController.getRightBumper()) {
      m_robotContainer.m_robotDrive.setX();
    }

    if(m_robotContainer.m_driveController.getLeftStickButton()) {
      m_robotContainer.m_robotDrive.zero();
    }

    if(m_robotContainer.m_driveController.getLeftTriggerAxis() >= 0.1) {
      teleopTurbo = 1.0 - m_robotContainer.m_driveController.getLeftTriggerAxis() * 0.8;
    } else {
      teleopTurbo = 1.0 - m_robotContainer.m_driveController.getRightTriggerAxis() * 0.8;
    }
    
    double shooterPower = 0;
    double shooterAcceleratorPower = 0;
     
    if(m_robotContainer.m_clawController.getAButton()) {
      m_robotContainer.m_intake.set(0.6);
      m_robotContainer.m_intakeContraRoller.set(0.6);
      
      shooterAcceleratorPower -= 0.3;
      shooterPower -= 0.3;
    } else if(m_robotContainer.m_clawController.getBButton()) {
      m_robotContainer.m_intake.set(-0.4);
      m_robotContainer.m_intakeContraRoller.set(-1.0);
    } else if(m_robotContainer.m_clawController.getLeftBumper()) {
      m_robotContainer.m_intake.set(0.3);
      m_robotContainer.m_intakeContraRoller.set(0.3);
    } else if(m_robotContainer.m_clawController.getRightBumper()) {
      m_robotContainer.m_intake.set(-0.3);
      m_robotContainer.m_intakeContraRoller.set(-0.3);
    } else {
      m_robotContainer.m_intake.set(0);
      m_robotContainer.m_intakeContraRoller.set(0);
    }
    
    if(m_robotContainer.m_driveController.getRightTriggerAxis() > 0.5) {
      shooterAcceleratorPower += 0.5;
    }
    
    if(m_robotContainer.m_clawController.getLeftTriggerAxis() > 0.5) {
      shooterPower += 0.2;
    }

    if(m_robotContainer.m_driveController.getYButton() || m_robotContainer.m_clawController.getRightTriggerAxis() > 0.5 || m_robotContainer.m_clawController.getYButton()) {
      shooterPower += 1.0;
    }

    m_robotContainer.m_shooterLeft.set(shooterPower);
    m_robotContainer.m_shooterRight.set(shooterPower);
    m_robotContainer.m_shooterAccelerator.set(shooterAcceleratorPower);

    if(m_robotContainer.m_clawController.getPOV() == 90) {
      m_robotContainer.m_hangRelease.set(1.0); 
    } else if(m_robotContainer.m_clawController.getPOV() == 270) {
      m_robotContainer.m_hangRelease.set(0.0);
    }

    // m_robotContainer.m_hang.set(-m_robotContainer.m_clawController.getLeftY());
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
