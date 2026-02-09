package frc.robot;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.XboxController;

import com.revrobotics.spark.SparkMax;

public class RobotMap {
    public static SparkMax TopLeft = new SparkMax(0, MotorType.kBrushless); 
    public static SparkMax TopRight = new SparkMax(0, MotorType.kBrushless); 
    public static SparkMax BottomLeft = new SparkMax(0, MotorType.kBrushless); 
    public static SparkMax BottomRight = new SparkMax(0, MotorType.kBrushless); 

    public static SparkMax Intake = new SparkMax(0, MotorType.kBrushless);
    public static SparkMax Shooter = new SparkMax(0, MotorType.kBrushless);


    public static XboxController Controller = new XboxController(0);
}
