package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Drive extends SubsystemBase {
    private SparkMax TopLeft = RobotMap.TopLeft;
    private SparkMax TopRight =  RobotMap.TopRight;

    public XboxController Controller = RobotMap.Controller;

    private DifferentialDrive DD = new DifferentialDrive(TopLeft, TopRight);

    @Override
    public void periodic() {
        double rt = Controller.getRightTriggerAxis();
        double lt = Controller.getLeftTriggerAxis();
        double LeftStick = Controller.getLeftX();
        double speed;

        if (Controller.getLeftTriggerAxis()>0.05) speed = Controller.getLeftTriggerAxis(); else{
            if (Controller.getRightTriggerAxis()>0.05) speed = -Controller.getRightTriggerAxis(); else speed = 0;
        }

        DD.arcadeDrive(speed,LeftStick);

        super.periodic();
    }
}