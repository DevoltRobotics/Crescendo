package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;

public class Auto3NotasDeLado {

    public static void run(RobotContainer m_robotContainer) {
        m_robotContainer.m_robotDrive.resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(125)));

        new InstantCommand(() -> {
            m_robotContainer.m_shooterLeft.set(1.0);
            m_robotContainer.m_shooterRight.set(1.0);
        }).andThen(
                m_robotContainer.m_robotDrive.getGoToPointCommand(new Pose2d(0.38, -0.2, Rotation2d.fromDegrees(125)), 1.0, 3))
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

                            m_robotContainer.m_intake.set(0.6);
                            m_robotContainer.m_intakeContraRoller.set(0.6);
                        }))
                .andThen(
                        m_robotContainer.m_robotDrive.getGoToPointCommand(new Pose2d(0.3, -0.35, Rotation2d.fromDegrees(0)),
                                0.8, 5))
                .andThen(
                        m_robotContainer.m_robotDrive.getGoToPointCommand(new Pose2d(2.1, -0.25, Rotation2d.fromDegrees(0)),
                                0.7, 5))
                .andThen(
                        new WaitCommand(0.05))
                .andThen(
                        new InstantCommand(() -> {
                            m_robotContainer.m_intake.set(0.0);
                            m_robotContainer.m_intakeContraRoller.set(0.0);

                            m_robotContainer.m_armSolenoid.set(Value.kReverse);
                        }))
                .andThen(
                        m_robotContainer.m_robotDrive.getGoToPointCommand(new Pose2d(0.15, -0.35, Rotation2d.fromDegrees(125)), 
                                1.0, 3))
                .andThen(
                        new InstantCommand(() -> {
                            m_robotContainer.m_intake.set(0.6);
                            m_robotContainer.m_intakeContraRoller.set(0.6);

                            m_robotContainer.m_shooterAccelerator.set(-0.3);
                            m_robotContainer.m_shooterLeft.set(-0.3);
                            m_robotContainer.m_shooterRight.set(-0.3);
                        }))
                .andThen(
                        new WaitCommand(1.1))
                .andThen(
                        new InstantCommand(() -> {
                            m_robotContainer.m_intake.set(0.0);
                            m_robotContainer.m_intakeContraRoller.set(0.0);

                            m_robotContainer.m_shooterAccelerator.set(0);
                            m_robotContainer.m_shooterLeft.set(1.0);
                            m_robotContainer.m_shooterRight.set(1.0);

                            m_robotContainer.m_armSolenoid.set(Value.kForward);
                        }))
                .andThen(new WaitCommand(0.5))
                .andThen(
                        new InstantCommand(() -> {
                            m_robotContainer.m_shooterAccelerator.set(0.5);
                        }))
                .andThen(
                        new WaitCommand(1))
                .andThen(
                        new InstantCommand(() -> {
                            m_robotContainer.m_shooterAccelerator.set(0.0);
                            m_robotContainer.m_shooterLeft.set(0);
                            m_robotContainer.m_shooterRight.set(0);
                        }))
                .andThen(
                        m_robotContainer.m_robotDrive.getGoToPointCommand(new Pose2d(2, 0.3, Rotation2d.fromDegrees(90)), 0.7, 5))
                .schedule();
    }

}
