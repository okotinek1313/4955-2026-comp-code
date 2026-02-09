package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Manipulator extends SubsystemBase {
    private SparkMax Neo = RobotMap.Intake;
    private SparkMax Vortex = RobotMap.Shooter;

    private double Velocity = Vortex.getAbsoluteEncoder().getVelocity(); // this will be a V in rpm
    private double TargetVelocity = 6.1; //this is in M/S
    private double R = 0.0508; //this is in M
    private double RPM = (60*TargetVelocity)/2*Math.PI*R;

    private SparkClosedLoopController vortexController = Vortex.getClosedLoopController();

    private XboxController controller = new XboxController(0);

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Velocity", Velocity);
        
        //Shooter
        if (controller.getLeftBumper()) {
           shooting(); 
        } else {
           Vortex.stopMotor();
        }

        // Intake
        if (controller.getRightBumper()) {
            Vortex.set(1);
        } else {
            Vortex.stopMotor();
        }
        
        super.periodic();
    }

    private void shooting(){
        Neo.set(-1);
        vortexController.setSetpoint(RPM, ControlType.kVelocity);
    }

}
