package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.networktables.StringArrayTopic;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Drive extends SubsystemBase {
    private static SparkMax TopLeft = RobotMap.TopLeft;
    private static SparkMax TopRight =  RobotMap.TopRight;

    private static MotorController motorController_top_left = new MotorController() {
        @Override
        public void set(double speed) {
            TopLeft.set(speed);
        }

        @Override
        public double get() {
            return TopLeft.get();
        }

        @Override
        public void setInverted(boolean isInverted) {
            // TODO Auto-generated method stub
            throw new UnsupportedOperationException("Unimplemented method 'setInverted'");
        }

        @Override
        public boolean getInverted() {
            // TODO Auto-generated method stub
            throw new UnsupportedOperationException("Unimplemented method 'getInverted'");
        }

        @Override
        public void disable() {
            // TODO Auto-generated method stub
            throw new UnsupportedOperationException("Unimplemented method 'disable'");
        }

        @Override
        public void stopMotor() {
            // TODO Auto-generated method stub
            throw new UnsupportedOperationException("Unimplemented method 'stopMotor'");
        }
    };

        private static MotorController motorController_top_right = new MotorController() {
        @Override
        public void set(double speed) {
            TopRight.set(speed);
        }

        @Override
        public double get() {
            return TopRight.get();
        }

        @Override
        public void setInverted(boolean isInverted) {
            // TODO Auto-generated method stub
            throw new UnsupportedOperationException("Unimplemented method 'setInverted'");
        }

        @Override
        public boolean getInverted() {
            // TODO Auto-generated method stub
            throw new UnsupportedOperationException("Unimplemented method 'getInverted'");
        }

        @Override
        public void disable() {
            // TODO Auto-generated method stub
            throw new UnsupportedOperationException("Unimplemented method 'disable'");
        }

        @Override
        public void stopMotor() {
            // TODO Auto-generated method stub
            throw new UnsupportedOperationException("Unimplemented method 'stopMotor'");
        }
    };

    public XboxController Controller = RobotMap.Controller;

    private DifferentialDrive DD = new DifferentialDrive(motorController_top_left, motorController_top_right);

    @Override
    public void periodic() {
        double rt = Controller.getRightTriggerAxis();
        double lt = Controller.getLeftTriggerAxis();
        double LeftStick = Controller.getLeftX();
        double speed;

        if (rt > 0.05) {speed = rt;} else {
            if (lt > 0.05) {speed = -lt;} else {speed = 0;}
        }

        DD.arcadeDrive(speed,LeftStick);

        super.periodic();
    }
}