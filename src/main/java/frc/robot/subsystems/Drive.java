package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.revrobotics.spark.SparkBase.ControlType;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.LimelightHelpers;
import frc.robot.RobotMap;
import frc.robot.RobotMap.OperatorConstants;

public class Drive extends SubsystemBase{
    
    SparkFlex frontRight = RobotMap.frontRight;
    SparkFlex frontLeft = RobotMap.frontLeft;
    SparkFlex backRight = RobotMap.backRight;
    SparkFlex backLeft = RobotMap.backleft;
    public DifferentialDrive DD;
    public double throttle = 0;
    public double steer = 0;
    XboxController controller = new XboxController(OperatorConstants.kDriverControllerPort);
    public AHRS gyro = RobotMap.gyro;
    final static Field2d field = new Field2d();
    final double trackwidth = 0.635;//In meters
    final double radiusowheels = 0.0762;//in meters
    public final int gearratio = 10;
    static int roitation = 0;
    SparkClosedLoopController leftController;
    SparkClosedLoopController righController;
    PIDController gyroController = new PIDController(0.006, 0, 0);

        Pose2d pose;
  StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault()
  .getStructTopic("RobotPose", Pose2d.struct).publish();

  StructPublisher<Pose2d> publisher2 = NetworkTableInstance.getDefault()
  .getStructTopic("Limelight see", Pose2d.struct).publish();
  StructPublisher<Pose2d> publisher3 = NetworkTableInstance.getDefault()
  .getStructTopic("Where robot should be(auto)", Pose2d.struct).publish();


      
DifferentialDriveKinematics kinematics =
new DifferentialDriveKinematics(trackwidth);

DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds(
  ((frontLeft.getEncoder().getVelocity()/60)/gearratio)*(radiusowheels*2*Math.PI),
 ((frontRight.getEncoder().getVelocity()/60)/gearratio)*(radiusowheels*2*Math.PI));

// Convert to chassis speeds.

public ChassisSpeeds chassisSpeeds = kinematics.toChassisSpeeds(wheelSpeeds);
// Linear velocity
double linearVelocity = chassisSpeeds.vxMetersPerSecond;
// Angular velocity
double angularVelocity = chassisSpeeds.omegaRadiansPerSecond;
LimelightHelpers.PoseEstimate mt2;

DifferentialDrivePoseEstimator m_poseestimator = new DifferentialDrivePoseEstimator(kinematics,
  new Rotation2d(),
 (frontLeft.getEncoder().getPosition()/gearratio)*(radiusowheels*2*Math.PI), 
 (frontRight.getEncoder().getPosition()/gearratio)*(radiusowheels*2*Math.PI),
  new Pose2d(5.0, 0, gyro.getRotation2d()));

SysIdRoutine routine;
    public Drive(){

  

        DD = new DifferentialDrive(frontLeft, frontRight);

        DD.setSafetyEnabled(false);
      leftController = frontLeft.getClosedLoopController();
      righController = frontRight.getClosedLoopController();

      gyroController.setTolerance(5);
      gyroController.enableContinuousInput(-180,180);
      

      RobotConfig config;
    try{
      config = RobotConfig.fromGUISettings();




        // Configure AutoBuilder last
    AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotSpeed, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> DriveRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPLTVController(0.02), // PPLTVController is the built in path following controller for differential drive trains
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
            PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            field.getObject("target pose").setPose(pose);
        });
        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            // Do whatever you want with the poses here
            field.getObject("path").setPoses(poses);
        });

    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }
  

      // Creates a SysIdRoutine
    routine = new SysIdRoutine(
    new SysIdRoutine.Config(null,Volts.of(3),null),
    new SysIdRoutine.Mechanism((e)->driveVoltage(e), null, this)
);
    }

    public Command quasForward(){
      System.out.println("Quasiatic Foward");
      
      return routine.quasistatic(Direction.kForward);
    }
    public Command quasBack(){
      System.out.println("Quasiatic reverse");
      return routine.quasistatic(Direction.kReverse);
    }
    public Command dynaBack(){
      System.out.println("Static reverse");
      return routine.dynamic(Direction.kReverse);
    }
    public Command dynaForward(){
      System.out.println("Static Forward");
      return routine.dynamic(Direction.kForward);
    }

    @Override
    public void periodic() {
        

      SmartDashboard.putBoolean("Gyro connecting?", gyro.isConnected());
      SmartDashboard.putBoolean("Gyro calibrating?", gyro.isCalibrating());
      SmartDashboard.putNumber("Yaw", gyro.getYaw());

      SmartDashboard.putNumber("TX", LimelightHelpers.getTX(""));
      SmartDashboard.putBoolean("TV", LimelightHelpers.getTV(""));


      
 pose = m_poseestimator.update(gyro.getRotation2d(),
 (frontLeft.getEncoder().getPosition()/gearratio)*(radiusowheels*2*Math.PI),
(frontRight.getEncoder().getPosition()/gearratio)*radiusowheels*2*Math.PI);
 if(SmartDashboard.getBoolean("Use Limelight to update pose", false)){


if(SmartDashboard.getBoolean("Use Limelight to update pose", false)){


   LimelightHelpers.SetRobotOrientation("", m_poseestimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
  mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("");



  if (mt2 != null) {
    
  
    boolean doRejectUpdate = false;
  // if our angular velocity is greater than 360 degrees per second, ignore vision updates
  if(Math.abs(gyro.getRate()) > 360)
  {
    doRejectUpdate = true;
  }
  if(mt2.tagCount == 0)
  {
    doRejectUpdate = true;
  }
  if(!doRejectUpdate)
  {
    publisher2.set(mt2.pose);
    m_poseestimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
    m_poseestimator.addVisionMeasurement(
        mt2.pose,
        mt2.timestampSeconds);
  }}
}}

field.setRobotPose(pose);
 publisher.set(pose);
 
 SmartDashboard.putString("Pose", pose.toString());
 SmartDashboard.putData("Field", field);


    }
    
    
    public void runDD(){
      
        if (controller.getLeftTriggerAxis()>0.05) {
            throttle = -controller.getLeftTriggerAxis();
        }else{

        if (controller.getRightTriggerAxis()>0.05) {
            throttle = controller.getRightTriggerAxis();
        }else{
            throttle = 0;
        }}
        steer = -controller.getLeftX();

        //quik lielight thing
         
        if (controller.getLeftBumperButton()) {
        //look();
        //DD.arcadeDrive(0.11, 0);
          turntoAngle(90);
        }else{
        DD.arcadeDrive(throttle, steer);
        }
    }
    public void look(){
      if (LimelightHelpers.getTV("")) {
        double TX = LimelightHelpers.getTX("");
      DD.arcadeDrive(0, -(TX/60));
      }

    }
    //FOr PathPlanner
    Pose2d getPose(){
      return pose;
    }

    void resetPose(Pose2d npose){
      pose = npose;
      m_poseestimator.resetPose(npose);
    }
    ChassisSpeeds getRobotSpeed(){

      DifferentialDriveWheelSpeeds wheelSpeedstemp = new DifferentialDriveWheelSpeeds(
        ((frontLeft.getEncoder().getVelocity()/60)/gearratio)*(radiusowheels*2*Math.PI),
       ((frontRight.getEncoder().getVelocity()/60)/gearratio)*(radiusowheels*2*Math.PI));

      return kinematics.toChassisSpeeds(wheelSpeedstemp);
    }

    void DriveRelative(ChassisSpeeds speeds){
      DifferentialDriveWheelSpeeds tempkin = kinematics.toWheelSpeeds(speeds);
      leftController.setSetpoint(tempkin.leftMetersPerSecond*60*gearratio/(radiusowheels*2*Math.PI), ControlType.kVelocity);
      righController.setSetpoint(tempkin.rightMetersPerSecond*60*gearratio/(radiusowheels*2*Math.PI), ControlType.kVelocity);

    }
    void driveVoltage(Voltage voltage){
      System.out.println(voltage);
      frontLeft.setVoltage(voltage);
      backLeft.setVoltage(voltage);
      frontRight.setVoltage(voltage);
      backRight.setVoltage(voltage);
    }

    boolean turntoAngle(double angle){
      
    double out = -gyroController.calculate(gyro.getYaw(),angle);
      SmartDashboard.putNumber("pidOUT", out);
      DD.arcadeDrive(0, out);

      return false;      
    }
}
