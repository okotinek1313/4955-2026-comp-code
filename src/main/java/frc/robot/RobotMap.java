// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class RobotMap {

  public static SparkFlex frontLeft = new SparkFlex(2, MotorType.kBrushless);
  public static SparkFlex backleft= new SparkFlex(3, MotorType.kBrushless);
  public static SparkFlex frontRight= new SparkFlex(1, MotorType.kBrushless);
  public static SparkFlex backRight= new SparkFlex(4, MotorType.kBrushless);



  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
