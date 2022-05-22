// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.invoke.ConstantBootstraps;

import com.ctre.phoenix.sensors.PigeonIMUConfiguration;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.swervemath.*;
import frc.swervemath.Hardware.*;
import frc.swervemath.math.*;


public class DriveTrain extends SubsystemBase {
  private final PS4Controller controller = new PS4Controller(0);
  private final frc.swervemath.DriveTrain DriveTrain;

  public DriveTrain() {
    DriveTrain = new frc.swervemath.DriveTrain(new PigeonInterface(Constants.IMUCANid), 
                                               new Wheel(new WCPSwerve(20, 1, 10, 0), new Vector2(-1.05f, -1.3f)),
                                               new Wheel(new WCPSwerve(2, 3, 11, 1), new Vector2(1.05f, -1.3f)),
                                               new Wheel(new WCPSwerve(4, 5, 12, 2), new Vector2(-1.05f, 1.3f)),
                                               new Wheel(new WCPSwerve(6, 7, 13, 3), new Vector2(1.05f, 1.3f)));

    DriveTrain.getGyro().zero();
  }

  @Override
  public void periodic() {
    DriveTrain.fieldOrientedDrive(new Vector2(-(float)controller.getRawAxis(0), -(float)controller.getRawAxis(1)), (float)controller.getRawAxis(2));
  }
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
