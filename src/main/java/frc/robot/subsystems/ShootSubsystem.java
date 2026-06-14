// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;

public class ShootSubsystem extends SubsystemBase {

  private static final int shooterMotorCANId = 10;

  private static final int advance1MotorCANId = 13;
  private static final int advance2MotorCANId = 14;
  private static final int hoodRotateMotorCANId = 17;

  private SparkFlex shootMotor;

  private SparkFlex advance1Motor;
  private SparkFlex advance2Motor;

  private SparkFlex hoodRotateMotor;
  private PIDController hoodEncoderPidController;
  private AbsoluteEncoder hoodEncoder = null;

  public double velocity = 0;

  public static enum HoodPosition {
    DOWN(0.019),
    DEPLOY_CLIMB(.165),
    CLIMB(.056),
    MAX(0.165);

    public final double value;

    HoodPosition(double v) {
      value = v;
    }
  };

  private double hoodPosition = 0.078;

  public ShootSubsystem() {
    shootMotor = new SparkFlex(shooterMotorCANId, MotorType.kBrushless);

    advance1Motor = new SparkFlex(advance1MotorCANId, MotorType.kBrushless);
    advance2Motor = new SparkFlex(advance2MotorCANId, MotorType.kBrushless);

    hoodRotateMotor = new SparkFlex(hoodRotateMotorCANId, MotorType.kBrushless);

    // hood stuff.
    hoodEncoderPidController = new PIDController(12, 0, .5);
    hoodEncoderPidController.enableContinuousInput(0, 1);
    hoodEncoder = hoodRotateMotor.getAbsoluteEncoder();
  }

  @Override
  public void periodic() {
    setHood();
  }

  private void setHood() {
    hoodEncoderPidController.setSetpoint(hoodPosition);
    double speed = hoodEncoderPidController.calculate(hoodEncoder.getPosition());
    hoodRotateMotor.set(speed);
  }

  public void setAdvanceOn() {
    advance1Motor.set(.75);
    advance2Motor.set(1);
  }

  public void setAdvanceOff() {
    advance1Motor.set(0);
    advance2Motor.set(0);
  }

  @AutoLogOutput(key = "Shooter/RPM")
  public double getVelocity() {
    return shootMotor.getEncoder().getVelocity();
  }

  @AutoLogOutput(key = "Shooter/Voltage")
  public double getVoltage() {
    return shootMotor.getBusVoltage();
  }

  // public double getShooterVelocity(int shooter) {
  // return shootMotor.getEncoder().getVelocity();
  // }

  public void setShootSpeed(double speed) {
    shootMotor.set(speed);
  }

  public void setVelocity(double velocity) {
    this.velocity = velocity;
  }

  // Feed Forward calculation based on target velocity and empirically determined kFF constant.
  private double kFF = 1.05 * 12.0 / 6784;

   private double calculateFeedForward() {
         return kFF * velocity;
  }
}
