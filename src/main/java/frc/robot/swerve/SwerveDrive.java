package frc.robot.swerve;

import java.util.ArrayList;
import java.util.List;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.sim.SimGyroSensorModel;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;
import edu.wpi.first.math.numbers.N7;
import edu.wpi.first.wpilibj.SPI;


public class SwerveDrive implements Loggable {
  public QuadFalconSwerveDrive quadFalconSwerveDrive;
  public boolean autoLimeLightAim = false;
  public boolean acceleratedInputs = true;
  public boolean defensiveStop = true;

  public double previousXDistance = 0;
  public double previousYDistance = 0;
  public double previousTimestamp = 0;
  private double SDxSpeed=0;
  private double SDySpeed=0;
  private double SDrotation=0;


  public ArrayList<Double> velocities = new ArrayList<Double>();
  private static SwerveDrive SINGLE_INSTANCE = new SwerveDrive();
  
  public boolean SDFieldRelative= SwerveConstants.DEFAULT_FIELD_RELATIVE_DRIVE;
  public boolean holdRobotAngleEnabled = SwerveConstants.DEFAULT_HOLD_ROBOT_ANGLE;
  public PIDController holdRobotAngleController = new PIDController(SwerveConstants.ROBOTHoldAngleKP, 0, 0);
  
  public double holdRobotAngleSetpoint = SwerveConstants.DEFAULT_HOLD_ROBOT_ANGLE_SETPOINT;
  public double joystickDriveGovernor = SwerveConstants.SPEED_GOVERNOR;

  public SwerveDrivePoseEstimator<N7, N7, N5> m_poseEstimator;
  public Pose2d prevRobotPose = new Pose2d();

  public static AHRS GYRO;
  public SimGyroSensorModel simNavx; 

  
  public static SwerveDrive getInstance() {
    return SINGLE_INSTANCE;
  }

  public void init(){
    quadFalconSwerveDrive = new QuadFalconSwerveDrive();

    SwerveDrive.GYRO = new AHRS(SPI.Port.kMXP);
    SwerveDrive.GYRO.reset(); 
    simNavx = new SimGyroSensorModel();

    velocities.add(0.0);
    velocities.add(0.0);
    holdRobotAngleController.disableContinuousInput();
    holdRobotAngleController.setTolerance(Math.toRadians(2));
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
    m_poseEstimator = 
    new SwerveDrivePoseEstimator<N7, N7, N5>(
        Nat.N7(),
        Nat.N7(),
        Nat.N5(),
        getRobotAngle(),
        quadFalconSwerveDrive.getModulePositions(),
        new Pose2d(),
        quadFalconSwerveDrive.m_kinematics,
        VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5), 0.05, 0.05, 0.05, 0.05),
        VecBuilder.fill(Units.degreesToRadians(0.01), 0.01, 0.01, 0.01, 0.01),
        VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));
  }

  public void swervePeriodic() {

    joystickDrive();
    drive(
      getSDxSpeed(), 
      getSDySpeed(), 
      getSDRotation(), 
      getSDFieldRelative());
    
  }

  public void simulationPeriodic(){
      for(SwerveModule module : quadFalconSwerveDrive.SwerveModuleList) {
        module.simModule.simulationPeriodic();
      }
    simNavx.update(m_poseEstimator.getEstimatedPosition(), prevRobotPose );
    }

  /**
  * Method to drive the robot using the following params
  *
  * @param _xSpeed Speed of the robot in the x direction (forward).
  * @param _ySpeed Speed of the robot in the y direction (sideways).
  * @param _rot Angular rate of the robot.
  * @param _fieldRelative Whether the provided x and y speeds are relative to the field.
  */
  @SuppressWarnings("ParameterName")
  public void drive(double _xSpeed, double _ySpeed, double _rot, boolean _fieldRelative) {
    if (Robot.xbox.getRightStickButton() || autoLimeLightAim){
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
      _rot = holdRobotAngleController.calculate(getRobotAngle().getRadians(), ((getRobotAngleDegrees() - limelightTX())/360)*(2*Math.PI));
      holdRobotAngleSetpoint = getRobotAngle().getRadians();
    } else if (_xSpeed == 0 && _ySpeed == 0 && _rot == 0 && holdRobotAngleEnabled) {
      System.out.println("Evan sucks booty%");
    
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(1);
    } else if (Robot.xbox.getLeftStickButton()){
      _fieldRelative = false;
    } else {
      holdRobotAngleSetpoint = getRobotAngle().getRadians();
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
    }

    SwerveModuleState[] moduleStates =
          quadFalconSwerveDrive.m_kinematics.toSwerveModuleStates( _fieldRelative ? 
          ChassisSpeeds.fromFieldRelativeSpeeds(_xSpeed, _ySpeed, _rot, getRobotAngle())
          : new ChassisSpeeds(_xSpeed, _ySpeed, _rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, SwerveConstants.MAX_SPEED_METERSperSECOND);
      
    if (defensiveStop && _xSpeed == 0 && _ySpeed == 0 && _rot == 0) {
      quadFalconSwerveDrive.activateDefensiveStop();
    } else {
      quadFalconSwerveDrive.setModuleSpeeds(moduleStates);
    }
  }

  /**This ONLY saves speeds.  You must also call the drive method */  
  public void joystickDrive(){
    double x;
    double y;
    double rot;

    if (acceleratedInputs) {
      x = -Robot.xbox.getLeftY();
      y = -Robot.xbox.getLeftX();
      rot = -Robot.xbox.getRightX();
    } else {
      x = -Math.signum(Robot.xbox.getLeftY()) * Math.sqrt(Math.abs(Robot.xbox.getLeftY()));
      y = -Math.signum(Robot.xbox.getLeftX()) * Math.sqrt(Math.abs(Robot.xbox.getLeftX()));
      rot = -Math.signum(Robot.xbox.getRightX()) * Math.sqrt(Math.abs(Robot.xbox.getRightX()));
    }

    SDxSpeed = convertToMetersPerSecond(deadband(x))*joystickDriveGovernor;
    SDySpeed = convertToMetersPerSecond(deadband(y))*joystickDriveGovernor;
    SDrotation = convertToRadiansPerSecond(deadband(rot))*joystickDriveGovernor;
    //System.out.println(SDrotation);
    
  }



  /**
   * Adjust the measurement noise/trust of vision estimation as robot velocities change.
   */
  private Vector<N3> calculateVisionNoise(){
    ChassisSpeeds speeds = quadFalconSwerveDrive.m_kinematics.toChassisSpeeds(quadFalconSwerveDrive.getModuleStates());
    double linearPercent = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) / (
        SwerveConstants.MAX_SPEED_METERSperSECOND);
    double angularPercent = Math.abs(speeds.omegaRadiansPerSecond) / SwerveConstants.MAX_SPEED_RADIANSperSECOND;
    return VecBuilder.fill(
        MathUtil.interpolate(0.05, 10, linearPercent),
        MathUtil.interpolate(0.05, 10, linearPercent),
        Units.degreesToRadians(MathUtil.interpolate(1, 45, angularPercent))
    );
  }


  public Rotation2d getRobotAngle(){
    if (GYRO.isConnected()){
        return GYRO.getRotation2d();
    } else {
        try {
        //System.out.println( deltaTime);
        return Robot.SWERVEDRIVE.m_poseEstimator.getEstimatedPosition().getRotation().rotateBy(new Rotation2d(quadFalconSwerveDrive.m_kinematics.toChassisSpeeds(quadFalconSwerveDrive.getModuleStates()).omegaRadiansPerSecond *Robot.deltaTime));
        } catch (Exception e) {
        return new Rotation2d();        
        }
    }
  }




  public double limelightTX() {  
    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    return tx;
  } //Testing kP=1.5

/**Sets the robots speed parameters to zero */
  public void zeroSwerveDrive(){
    SDxSpeed = 0;
    SDySpeed = 0;
    SDrotation = 0;
  }

  private double convertToMetersPerSecond(double _input){
    return _input*SwerveConstants.MAX_SPEED_METERSperSECOND;
  }

  private double convertToRadiansPerSecond(double _input){
    return _input*SwerveConstants.MAX_SPEED_RADIANSperSECOND;
  }
  private double deadband(double _input){
      if(Math.abs(_input)<= Constants.XBOXDEADBAND){
        _input = 0;
      }
      return _input;
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry(){
    m_poseEstimator.update(getRobotAngle(), 
    quadFalconSwerveDrive.getModuleStates(), 
    quadFalconSwerveDrive.getModulePositions());
  }



  /**
   * A convenience method to draw the robot pose and 4 poses representing the wheels onto the field2d.
   * @param field
   */
  public void drawRobotOnField(Field2d field) {
    field.setRobotPose(getPose());
    
    // Draw a pose that is based on the robot pose, but shifted by the translation of the module relative to robot center,
    // then rotated around its own center by the angle of the module.
    
    field.getObject("frontLeft").setPose(
        getPose().transformBy(new Transform2d(quadFalconSwerveDrive.FrontLeftSwerveModule.mTranslation2d, quadFalconSwerveDrive.FrontLeftSwerveModule.getSwerveModuleState().angle)));
    field.getObject("frontRight").setPose(
        getPose().transformBy(new Transform2d(quadFalconSwerveDrive.FrontRightSwerveModule.mTranslation2d, quadFalconSwerveDrive.FrontRightSwerveModule.getSwerveModuleState().angle)));
    field.getObject("backLeft").setPose(
        getPose().transformBy(new Transform2d(quadFalconSwerveDrive.BackLeftSwerveModule.mTranslation2d, quadFalconSwerveDrive.BackLeftSwerveModule.getSwerveModuleState().angle)));
    field.getObject("backRight").setPose(
        getPose().transformBy(new Transform2d(quadFalconSwerveDrive.BackRightSwerveModule.mTranslation2d, quadFalconSwerveDrive.BackRightSwerveModule.getSwerveModuleState().angle)));
  }

  /**
   * Return the current position of the robot on field
   * Based on drive encoder and gyro reading
   */
  public Pose2d getPose() {
        return m_poseEstimator.getEstimatedPosition();
  }
  
  @Log.Gyro(name = "Robot Angle", rowIndex = 2, columnIndex = 5)
  private AHRS getGyro(){
    return SwerveDrive.GYRO;
  }

  public boolean getSDFieldRelative() {
    return SDFieldRelative;
  }

  public void setSDxSpeed(double _input) {
    SDxSpeed = _input;
  }
  public void setSDySpeed(double _input) {
    SDySpeed = _input;
  }
  public void setSDRotation(double _input) {
    SDrotation = _input;
  }
  @Log.NumberBar(min = -5, max = 5, rowIndex = 0, columnIndex = 7, height = 1, width = 1) 
  public double getSDxSpeed() {
    return SDxSpeed;
  }
  @Log.NumberBar(min = -5, max = 5, rowIndex = 0, columnIndex = 8,height = 1, width = 1)
  public double getSDySpeed(){
    return SDySpeed;
  }
  @Log.Dial(rowIndex = 0, columnIndex = 9, height = 1, width = 1)
  public double getSDRotation() {
    return SDrotation;
  }

  @Config(defaultValueBoolean = true)
  public void setAcceleratedInput(boolean _input) {
    acceleratedInputs = _input;
  }

  @Config.ToggleButton(name = "FieldOriented?", defaultValue = false, rowIndex = 1, columnIndex =0, height = 1, width = 2)
  public void setSDFieldRelative(boolean _input) {
    SDFieldRelative = _input;
  }

  @Config.ToggleButton(name = "Hold Robot Angle?", defaultValue = false, rowIndex = 0, columnIndex =0, height = 1, width = 2)
  public void setHoldAngleEnabled(boolean _boolean){
    holdRobotAngleEnabled = _boolean;
  }

  @Log.Dial(name= "Current Robot Angle", min = -180, max = 180, rowIndex = 0, columnIndex =3)
  public double getRobotAngleDegrees(){
    return getRobotAngle().getDegrees();
  }
  @Log.Dial(name= "Hold Angle Setpoint", min = -180, max = 180, rowIndex = 0, columnIndex =4)
  public double getHoldAngleSetpoint(){
    return Math.toDegrees(holdRobotAngleSetpoint);
  }

  public void setHoldRobotAngleSetpoint(double _holdRobotAngleSetpoint) {
    holdRobotAngleSetpoint = Math.toRadians(_holdRobotAngleSetpoint);
  }

  public void resetOdometry(){
    m_poseEstimator.resetPosition(getRobotAngle(), quadFalconSwerveDrive.getModulePositions(), new Pose2d());
  }

  public void resetOdometry(Pose2d _Pose2d, Rotation2d _Rotation2d){
    m_poseEstimator.resetPosition(_Rotation2d, quadFalconSwerveDrive.getModulePositions(), _Pose2d);
  }



  @Log(rowIndex = 0, columnIndex = 5, height = 1, width = 1)
  public double getXPos(){
    return m_poseEstimator.getEstimatedPosition().getTranslation().getX();
  }
  @Log(rowIndex = 0, columnIndex = 6, height = 1, width = 1)
  public double getYPos(){
    return m_poseEstimator.getEstimatedPosition().getTranslation().getY();
  }



 

}
