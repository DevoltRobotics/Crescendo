package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;

public class Auto3NotasAzul {

    public static void run(RobotContainer m_robotContainer) {
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

                            m_robotContainer.m_intake.set(0.6);
                            m_robotContainer.m_intakeContraRoller.set(0.6);
                        }))
                .andThen(
                        m_robotContainer.m_robotDrive.getGoToPointCommand(new Pose2d(0.3, 0, Rotation2d.fromDegrees(0)),
                                0.8, 5))
                .andThen(
                        m_robotContainer.m_robotDrive.getGoToPointCommand(new Pose2d(1.1, 0, Rotation2d.fromDegrees(0)),
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
                        m_robotContainer.m_robotDrive
                                .getGoToPointCommand(new Pose2d(0.32, 0.0, Rotation2d.fromDegrees(180)), 0.7, 5))
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

                            m_robotContainer.m_intake.set(0.6);
                            m_robotContainer.m_intakeContraRoller.set(0.6);
                        }))
                .andThen(
                        m_robotContainer.m_robotDrive
                                .getGoToPointCommand(new Pose2d(1.75, 0, Rotation2d.fromDegrees(90)), 0.8, 5))
                .andThen(
                        m_robotContainer.m_robotDrive
                                .getGoToPointCommand(new Pose2d(1.75, 1.2, Rotation2d.fromDegrees(90)), 0.7, 5))
                .andThen(
                        new InstantCommand(() -> {
                            m_robotContainer.m_intake.set(0.0);
                            m_robotContainer.m_intakeContraRoller.set(0.0);

                            m_robotContainer.m_armSolenoid.set(Value.kReverse);
                        }))
                .andThen(
                        m_robotContainer.m_robotDrive
                                .getGoToPointCommand(new Pose2d(0.43, 0.3, Rotation2d.fromDegrees(180)), 0.6, 5))
                .andThen(
                        new InstantCommand(() -> {
                            m_robotContainer.m_intake.set(0.6);
                            m_robotContainer.m_intakeContraRoller.set(0.6);

                            m_robotContainer.m_shooterAccelerator.set(-0.3);
                            m_robotContainer.m_shooterLeft.set(-0.3);
                            m_robotContainer.m_shooterRight.set(-0.3);
                        }))
                .andThen(
                        new WaitCommand(0.9))
                .andThen(
                        new InstantCommand(() -> {
                            m_robotContainer.m_intake.set(0.0);
                            m_robotContainer.m_intakeContraRoller.set(0.0);

                            m_robotContainer.m_shooterAccelerator.set(0);
                            m_robotContainer.m_shooterLeft.set(1.0);
                            m_robotContainer.m_shooterRight.set(1.0);

                            m_robotContainer.m_armSolenoid.set(Value.kForward);
                        }))
                .andThen(new WaitCommand(0.7))
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
                        }))
                .andThen(
                        m_robotContainer.m_robotDrive.getGoToPointCommand(new Pose2d(2, 0.3, Rotation2d.fromDegrees(90)), 0.7, 5))
                .schedule();
    }

}
