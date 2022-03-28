package frc.robot.Subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Configuration.Constants;
import frc.robot.Utilities.Limelight;
import frc.robot.Utilities.PathFollower;
import frc.robot.Utilities.VisionTargetState;

public class Drivetrain implements ISubsystem {
    
    private CANSparkMax _leftFrontMotor;
    private CANSparkMax _leftMiddleMotor;
    private CANSparkMax _leftRearMotor;
    private CANSparkMax _rightFrontMotor;
    private CANSparkMax _rightMiddleMotor;
    private CANSparkMax _rightRearMotor;
    private DifferentialDrive _robotDrive;
    private Encoder _leftDriveEncoder;
    private Encoder _rightDriveEncoder;
    private AHRS _gyro;
    private PathFollower _pathFollower;
    private SimpleMotorFeedforward _feedForward;
    private Limelight _limelight;
    
    // Odometry class for tracking robot pose
    private final DifferentialDriveOdometry _odometry;
    
    private PIDController _leftDriveVelocityPID;
    private PIDController _rightDriveVelocityPID;
    private PIDController _turnPID;

    private static Drivetrain _instance;

    private double _targetAngle = 7769;

    /**
     * Returns the singleton instance of the Drivetrain
     * @return - Instance of the DriveTrain
     */
    public static Drivetrain GetInstance()
    {
        if (_instance == null)
        {
            _instance = new Drivetrain();
        }
        return _instance;
    }

    /**
     * Constructor for the Drivetrain
     */
    public Drivetrain()
    {
        _leftFrontMotor = new CANSparkMax(Constants.kLeftFrontDriveDeviceId, MotorType.kBrushless);
        _leftRearMotor = new CANSparkMax(Constants.kLeftRearDriveDeviceId, MotorType.kBrushless);

        _rightFrontMotor = new CANSparkMax(Constants.kRightFrontDriveDeviceId, MotorType.kBrushless);
        _rightFrontMotor.setInverted(true);

        _rightRearMotor = new CANSparkMax(Constants.kRightRearDriveDeviceId, MotorType.kBrushless);
        _rightRearMotor.setInverted(true);

        _leftRearMotor.follow(_leftFrontMotor);
        _rightRearMotor.follow(_rightFrontMotor);

        if (Constants.kCompetitionRobot) {

           _leftMiddleMotor = new CANSparkMax(Constants.kLeftMiddleDriveDeviceId, MotorType.kBrushless);
           _rightMiddleMotor = new CANSparkMax(Constants.kRightMiddleDriveDeviceId, MotorType.kBrushless);
           _rightMiddleMotor.setInverted(true);
            
           _leftMiddleMotor.follow(_leftFrontMotor);
           _rightMiddleMotor.follow(_rightFrontMotor);
        }

        _leftFrontMotor.setSmartCurrentLimit(60);
        _leftMiddleMotor.setSmartCurrentLimit(60);
        _leftRearMotor.setSmartCurrentLimit(60);
        _rightFrontMotor.setSmartCurrentLimit(60);
        _rightMiddleMotor.setSmartCurrentLimit(60);
        _rightRearMotor.setSmartCurrentLimit(60);

        _leftFrontMotor.burnFlash();
        _leftMiddleMotor.burnFlash();
        _leftRearMotor.burnFlash();
        _rightFrontMotor.burnFlash();
        _rightMiddleMotor.burnFlash();
        _rightRearMotor.burnFlash();

        _gyro = new AHRS(Port.kMXP);

        _leftDriveEncoder = new Encoder(Constants.kLeftEncoderPortA, Constants.kLeftEncoderPortB, true);
        _leftDriveEncoder.setDistancePerPulse(Constants.kDriveDistancePerPulse);
        _leftDriveEncoder.setSamplesToAverage(5);

        _rightDriveEncoder = new Encoder(Constants.kRightEncoderPortA, Constants.kRightEncoderPortB, false);
        _rightDriveEncoder.setDistancePerPulse(Constants.kDriveDistancePerPulse);
        _leftDriveEncoder.setSamplesToAverage(5);

        _leftDriveVelocityPID = new PIDController(Constants.kPathFollowingkP, 0.0, 0.0);
        _rightDriveVelocityPID = new PIDController(Constants.kPathFollowingkP, 0.0, 0.0);

        _turnPID = new PIDController(0.067, 0.0, 0.0035);
        _turnPID.setTolerance(2);

        _robotDrive = new DifferentialDrive(_leftFrontMotor, _rightFrontMotor);
        
        resetEncoders();
        _odometry = new DifferentialDriveOdometry(_gyro.getRotation2d());
        

        _feedForward = new SimpleMotorFeedforward(Constants.ksVolts,
        Constants.kvVoltSecondsPerMeter,
        Constants.kaVoltSecondsSquaredPerMeter);
        _pathFollower = new PathFollower();

        _limelight = Limelight.getInstance();
    }
    
    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return _odometry.getPoseMeters();
    }

    /**
     * Updates the odometry Pose
     */
    public void updatePose() {
        _odometry.update(_gyro.getRotation2d(), _leftDriveEncoder.getDistance(), _rightDriveEncoder.getDistance());
    }

    /**
     * Returns the current wheel speeds of the robot.
     *
     * @return The current wheel speeds.
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(_leftDriveEncoder.getRate(), _rightDriveEncoder.getRate());
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        _odometry.resetPosition(pose, _gyro.getRotation2d());
    }

    /**
     * Resets the odometry.
     */
    public void resetOdometry()
    {
        resetEncoders();
        _odometry.resetPosition(_pathFollower.getStartingPose(), _gyro.getRotation2d());
    }

    /**
     * Controls the left and right sides of the drive directly with voltages.
     *
     * @param leftVolts  the commanded left output
     * @param rightVolts the commanded right output
     */
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        _rightFrontMotor.setIdleMode(IdleMode.kBrake);
        //_rightMiddleMotor.setIdleMode(IdleMode.kBrake);
        _rightRearMotor.setIdleMode(IdleMode.kBrake);
        _leftFrontMotor.setIdleMode(IdleMode.kBrake);
        //_leftMiddleMotor.setIdleMode(IdleMode.kBrake);
        _leftRearMotor.setIdleMode(IdleMode.kBrake);

        _leftFrontMotor.setVoltage(leftVolts);
        _rightFrontMotor.setVoltage(rightVolts);
        _robotDrive.feed();
    }
    
    /**
     * Controls the throttle and turn of the drive directly with voltages. Drives using IdleMode.kCoast
     *
     * @param throttle - Robot speed along the X axis. Value between 1 and -1; Forward is positive
     * @param turn - Robot rotation rate along the Z axis. Value between 1 and -1; Clockwise is positive
     */
    public void drive(double throttle, double turn)
    {
        _rightFrontMotor.setIdleMode(IdleMode.kCoast);
        //_rightMiddleMotor.setIdleMode(IdleMode.kCoast);
        _rightRearMotor.setIdleMode(IdleMode.kCoast);
        _leftFrontMotor.setIdleMode(IdleMode.kCoast);
        //_leftMiddleMotor.setIdleMode(IdleMode.kCoast);
        _leftRearMotor.setIdleMode(IdleMode.kCoast);

        if (Math.abs(throttle) <= .1) {
            setRampRate(0);
        } else {
            setRampRate(.25);
        }

        _robotDrive.arcadeDrive(throttle, turn);
    }

    /**
     * Sets the left and right motor ramp rate
     *
     * @param rate - The rate you would like to set the left and right motor
     */
    public void setRampRate(double rate)
    {
        _leftFrontMotor.setOpenLoopRampRate(rate);
        //_leftMiddleMotor.setOpenLoopRampRate(rate);
        _leftRearMotor.setOpenLoopRampRate(rate);
        _rightFrontMotor.setOpenLoopRampRate(rate);
        //_rightMiddleMotor.setOpenLoopRampRate(rate);
        _rightRearMotor.setOpenLoopRampRate(rate);
    }

    /**
     * Controls the left and right sides of the drive directly with voltages. Drives using IdleMode.kBrake
     *
     * @param leftVolts  the commanded left output
     * @param rightVolts the commanded right output
     */
    public void driveBrake(double throttle, double turn)
    {
        _rightFrontMotor.setIdleMode(IdleMode.kBrake);
        _leftFrontMotor.setIdleMode(IdleMode.kBrake);
        _robotDrive.arcadeDrive(throttle, turn);
    }
    
    /**
     * Resets the drive encoders to currently read a position of 0.
     */
    public void resetEncoders() {
        _leftDriveEncoder.reset();
        _rightDriveEncoder.reset();
    }

    /**
     * Resets the Drivertrain gyro
     */
    public void resetGyro() {
        _gyro.reset();
    }
    
    /**
     * Gets the average distance of the two encoders.
     *
     * @return the average of the two encoder readings
     */
    public double getAverageEncoderDistance() {
        return (_leftDriveEncoder.getDistance() + _rightDriveEncoder.getDistance()) / 2.0;
    }

    /**
     * Gets the left drive encoder.
     *
     * @return the left drive encoder
     */
    public Encoder getLeftEncoder() {
        return _leftDriveEncoder;
    }

    /**
     * Gets the right drive encoder.
     *
     * @return the right drive encoder
     */
    public Encoder getRightEncoder() {
        return _rightDriveEncoder;
    }

    /**
     * Sets the max output of the drive. Useful for scaling the drive to drive more
     * slowly.
     *
     * @param maxOutput the maximum output to which the drive will be constrained
     */
    public void setMaxOutput(double maxOutput) {
        _robotDrive.setMaxOutput(maxOutput);
    }

    /** Zeroes the heading of the robot. */
    public void zeroHeading() {
        _gyro.reset();
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        return _gyro.getRotation2d().getDegrees();
    }
        
    // public double getHeading() {
    //     return Math.IEEEremainder(_gyro.getAngle(), 360) * (-1.0);
    // }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return -_gyro.getRate();
    }
    
    /**
     * Returns a new trajectory config  
     * 
     * @param isReverse - Whether or not to return the config in reverse 
     * 
     * @return The trajectory config
     */
    public TrajectoryConfig getTrajectoryConfig(boolean isReverse)
    {
      var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(Constants.ksVolts,
                                       Constants.kvVoltSecondsPerMeter,
                                       Constants.kaVoltSecondsSquaredPerMeter),
            Constants.kDriveKinematics,
            10);
        TrajectoryConfig config =
        new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond,
                             Constants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(Constants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);
            config.setReversed(isReverse);
            return config;
    }

    /**
     * Sets path follower current path as Test Path
     */
    public void setTestPath()
    {
        _pathFollower.setTestPath(getTrajectoryConfig(false));
    }

    /**
     * Sets path follower current path as Drive Forward And Shoot Path
     */
    public void setDriveForwardAndShootPath()
    {
        _pathFollower.setDriveForwardAndShootPath(getTrajectoryConfig(false));
    }

    /**
     * Sets path follower current path as Collect wo From Terminal Path
     */
    public void setCollectTwoFromTerminalPath()
    {
        _pathFollower.setCollectTwoFromTerminalPath(getTrajectoryConfig(false), getPose());
    }

    /**
     * Sets path follower current path as Drive Back From Terminal Path
     */
    public void setDriveBackFromTerminalPath()
    {
        _pathFollower.setDriveBackFromTerminalPath(getTrajectoryConfig(true), getPose());
    }

    /**
     * Sets path follower current path as Five Ball Part One Path
     */
    public void setFiveBallPartOnePath()
    {
        _pathFollower.setFiveBallPartOnePath(getTrajectoryConfig(false));
    }

    /**
     * Sets path follower current path as Five Ball Part Two To Terminal Path
     */
    public void setFiveBallPartTwoToTerminalPath()
    {
        _pathFollower.setFiveBallPartTwoToTerminalPath(getTrajectoryConfig(false), getPose().getRotation());
    }

    /**
     * Sets path follower current path as Five Ball Part Two From Terminal Path
     */
    public void setFiveBallPartTwoFromTerminalPath()
    {
        _pathFollower.setFiveBallPartTwoFromTerminalPath(getTrajectoryConfig(true), getPose().getRotation());
    }

    /**
     * Sets path follower current path as Four Ball Far Part One Path
     */
    public void setFourBallFarPartOnePath()
    {
        _pathFollower.setFourBallFarPartOneTrajectory(getTrajectoryConfig(false));
    }

    /**
     * Sets path follower current as Four Ball Far Part Two Out Path
     */
    public void setFourBallFarPartTwoOutPath()
    {
        _pathFollower.setFourBallFarPartTwoOutPath(getTrajectoryConfig(false), getPose());
    }
    
    /**
     * Sets path follower current path as Four Ball Far Part Two Path
     */
    public void setFourBallFarPartTwoBackPath()
    {
        _pathFollower.setFourBallFarPartTwoBackPath(getTrajectoryConfig(true), getPose());
    }

    /**
     * Starts the path that has already been set
     * <p>
     * METHOD SHOULD BE CALLED ONLY AFTER A PATH HAS BEEN SET
     */
    public void startPath()
    {
        _leftDriveVelocityPID.reset();
        _rightDriveVelocityPID.reset();
        resetOdometry(_pathFollower.getStartingPose());
        _pathFollower.startPath();
    }

    /**
     * Uses the path follower current path to drive to the target location
     */
    public void followPath()
    {
        var target = _pathFollower.getPathTarget(getPose());
        SmartDashboard.putNumber("leftSpeed", getWheelSpeeds().leftMetersPerSecond);
        SmartDashboard.putNumber("rightSpeed", getWheelSpeeds().rightMetersPerSecond);
        SmartDashboard.putNumber("leftTargetSpeed", target.leftMetersPerSecond);
        SmartDashboard.putNumber("rightTargetSpeed", target.rightMetersPerSecond);

        var leftOutputTarget = _leftDriveVelocityPID.calculate(getWheelSpeeds().leftMetersPerSecond, target.leftMetersPerSecond);
        var rightOutputTarget = _rightDriveVelocityPID.calculate(getWheelSpeeds().rightMetersPerSecond, target.rightMetersPerSecond);

        var leftFeedForward = _feedForward.calculate(target.leftMetersPerSecond);
        var rightFeedForward = _feedForward.calculate(target.rightMetersPerSecond);

        tankDriveVolts(leftOutputTarget + leftFeedForward, rightOutputTarget + rightFeedForward);

    }

    /**
     * Returns whether or not the started path is finished.
     * @return if path finished
     */
    public boolean isPathFinished()
    {
      return _pathFollower.isPathFinished();
    }

    /**
     * If the limilight has a selected target it will return the output needed to get to the target
     * @return the output needed to get to the limilight target. If no target exist returns 0.
     */
    public double followTarget()
    {
      if (!_limelight.hasTarget())
      {
        return 0;
      }

      var limelightTargetAngle = _limelight.getAngleToTarget();

      var targetAngle = getHeading() - limelightTargetAngle;

      var output = _turnPID.calculate(getHeading(), targetAngle);

      SmartDashboard.putNumber("turnOutput", output);
      
      if (_turnPID.atSetpoint()) {
        return 0;
      }

      if (Math.abs(output) > 0.5)
      {
        if (output > 0)
        {
          output = 0.5;
        } else {
          output = -0.5;
        }
      }
      return -output;
    }
    
    /**
     * Uses the visionTargetState to return the output needed to reach target
     * @param visionTargetState - The visionTargetState to use in calculations
     * @return - output needed to reach target
     */
    public double followTarget(VisionTargetState visionTargetState)
    {
        if (_targetAngle == 7769) {
            _targetAngle = getHeading() - visionTargetState.getOffset();
        }

      var output = _turnPID.calculate(getHeading(), _targetAngle);

      if (Math.abs(output) > 0.35)
      {
        if (output > 0)
        {
          output = 0.35;
        } else {
          output = -0.35;
        }
      }
      return -output;
    }

    /**
     * Turns the robot to the desired angle using Drivetrain.driveBrake
     * @param angle - Angle you wish to turn to.
     */
    public void turnToAngle(double angle)
    {
        var output = _turnPID.calculate(getHeading(), angle);

        if (Math.abs(output) > 0.5)
        {
            if (output > 0)
            {
                output = 0.5;
            } else {
                output = -0.5;
            }
        }
        driveBrake(0, -output);
    }

    /**
     * Whether or not if the robot has finished its turn
     * @return - has robot finished its turn
     */
    public boolean isTurnFinished()
    {
        return _turnPID.atSetpoint();
    }

    /**
     * Resets the turn pid of the driveTrain
     */
    public void resetPID()
    {
        _turnPID.reset();
    }

    /**
     * Sets the targetAngle of driveTrain back to 7769
     */
    public void resetTargetAngle()
    {
        _targetAngle = 7769;
    }

    /**
     * Logs driveTrain data to SmartDashboard
     */
    public void LogTelemetry() {
        // TODO Auto-generated method stub
        SmartDashboard.putNumber("leftDriveDistance", _leftDriveEncoder.getDistance());
        SmartDashboard.putNumber("leftDriveRate", _leftDriveEncoder.getRate());
        SmartDashboard.putNumber("rightDriveDistance", _rightDriveEncoder.getDistance());
        SmartDashboard.putNumber("rightDriveRate", _rightDriveEncoder.getRate());
        SmartDashboard.putNumber("gyroHeading", getHeading());
    }

    /**
     * Reads the data from the SmartDashboard... But at the moment this function is empty so it does nothing.
     */
    public void ReadDashboardData() {
        // TODO Auto-generated method stub
        
    }
}
