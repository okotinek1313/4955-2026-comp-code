package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Manipulator extends SubsystemBase {
    private SparkMax Intake = RobotMap.Intake;
    private SparkMax Shooter = RobotMap.Shooter;

    private double speed = 0.4;

    private XboxController controller = new XboxController(0);

    @Override
    public void periodic() {
        
        //Shooter
        if (controller.getLeftBumper()) {
            Shooter.set(speed);
        } else {
            Shooter.stopMotor();
        }

        // Intake
        if (controller.getRightBumper()) {
            Intake.set(speed);
        } else {
            Intake.stopMotor();
        }
        
        super.periodic();
    }

}