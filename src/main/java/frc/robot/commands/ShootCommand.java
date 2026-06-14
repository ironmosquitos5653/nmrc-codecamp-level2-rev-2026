// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.proto.Photon;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.ShootSubsystem;
import frc.robot.subsystems.drive.Drive;

public class ShootCommand extends Command {
  private PIDController aimPidController;
  private ShootSubsystem m_shootSubsystem;
  private Drive m_driveSubsystem;

  public ShootCommand(ShootSubsystem shootSubsystem, Drive driveSubsystem) {
      m_shootSubsystem = shootSubsystem;
      m_driveSubsystem = driveSubsystem;
      addRequirements(m_shootSubsystem, m_driveSubsystem);

      aimPidController = new PIDController(4, 0, .1);
      aimPidController.enableContinuousInput(-Math.PI, Math.PI);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private double calculateAimSpeed() {
    var targetPose = PhotonVisionSubsystem.aprilTag24Pose2d;
    var currentPose = m_driveSubsystem.getPose();

    Rotation2d toTarget =
      new Rotation2d(
            Math.atan2(
                targetPose.getY() - currentPose.getY(), targetPose.getX() - currentPose.getX()));

    Rotation2d currentRot = currentPose.getRotation();

    double error = toTarget.minus(currentRot).getRadians();

    double output = aimPidController.calculate(error);

    return output;
  }

}
