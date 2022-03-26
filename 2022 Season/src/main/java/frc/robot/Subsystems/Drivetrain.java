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

    public static Drivetrain GetInstance()
    {
        if (_instance == null)
        {
            _instance = new Drivetrain();
        }
        return _instance;
    }

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

    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        _odometry.resetPosition(pose, _gyro.getRotation2d());
    }

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
        _rightMiddleMotor.setIdleMode(IdleMode.kBrake);
        _rightRearMotor.setIdleMode(IdleMode.kBrake);
        _leftFrontMotor.setIdleMode(IdleMode.kBrake);
        _leftMiddleMotor.setIdleMode(IdleMode.kBrake);
        _leftRearMotor.setIdleMode(IdleMode.kBrake);

        _leftFrontMotor.setVoltage(leftVolts);
        _rightFrontMotor.setVoltage(rightVolts);
        _robotDrive.feed();
    }
    
    public void drive(double throttle, double turn)
    {
        _rightFrontMotor.setIdleMode(IdleMode.kCoast);
        _rightMiddleMotor.setIdleMode(IdleMode.kCoast);
        _rightRearMotor.setIdleMode(IdleMode.kCoast);
        _leftFrontMotor.setIdleMode(IdleMode.kCoast);
        _leftMiddleMotor.setIdleMode(IdleMode.kCoast);
        _leftRearMotor.setIdleMode(IdleMode.kCoast);

        if (Math.abs(throttle) <= .1) {
            setRampRate(0);
        } else {
            setRampRate(.25);
        }
        _robotDrive.arcadeDrive(throttle, turn);
    }

    public void setRampRate(double rate)
    {
        _leftFrontMotor.setOpenLoopRampRate(rate);
        _leftMiddleMotor.setOpenLoopRampRate(rate);
        _leftRearMotor.setOpenLoopRampRate(rate);
        _rightFrontMotor.setOpenLoopRampRate(rate);
        _rightMiddleMotor.setOpenLoopRampRate(rate);
        _rightRearMotor.setOpenLoopRampRate(rate);
    }

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
    public void setTestPath()
    {
        _pathFollower.setTestPath(getTrajectoryConfig(false));
    }
    public void setDriveForwardAndShootPath()
    {
        _pathFollower.setDriveForwardAndShootPath(getTrajectoryConfig(false));
    }

    public void setCollectTwoFromTerminalPath()
    {
        _pathFollower.setCollectTwoFromTerminalPath(getTrajectoryConfig(false), getPose());
    }

    public void setDriveBackFromTerminalPath()
    {
        _pathFollower.setDriveBackFromTerminalPath(getTrajectoryConfig(true), getPose());
    }

    public void setFifthBallPath()
    {
        _pathFollower.setFifthBallPath(getTrajectoryConfig(false), getPose());
    }

    public void setFiveBallPartOnePath()
    {
        _pathFollower.setFiveBallPartOnePath(getTrajectoryConfig(false));
    }

    public void setFiveBallPartTwoToTerminalPath()
    {
        _pathFollower.setFiveBallPartTwoToTerminalPath(getTrajectoryConfig(false), getPose().getRotation());
    }

    public void setFiveBallPartTwoFromTerminalPath()
    {
        _pathFollower.setFiveBallPartTwoFromTerminalPath(getTrajectoryConfig(true), getPose().getRotation());
    }
    public void setFourBallFarPartOnePath()
    {
        _pathFollower.setFourBallFarPartOneTrajectory(getTrajectoryConfig(false));
    }
    public void setFourBallFarPartTwoOutPath()
    {
        _pathFollower.setFourBallFarPartTwoOutPath(getTrajectoryConfig(false), getPose());
    }
    
    public void setFourBallFarPartTwoBackPath()
    {
        _pathFollower.setFourBallFarPartTwoBackPath(getTrajectoryConfig(true), getPose());
    }

    public void startPath()
    {
        _leftDriveVelocityPID.reset();
        _rightDriveVelocityPID.reset();
        resetOdometry(_pathFollower.getStartingPose());
        _pathFollower.startPath();
    }
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
    public boolean isPathFinished()
    {
      return _pathFollower.isPathFinished();
    }

    public double followTarget()
    {
      if (!_limelight.hasTarget())
      {
        return 0;
      }

      var limelightTargetAngle = _limelight.getAngleToTarget();

      var targetAngle = getHeading() - limelightTargetAngle;
    //   if (Math.abs(targetAngle) <= 2)
    //   {
    //       targetAngle = getHeading();
    //   }

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

    public boolean isTurnFinished()
    {
        return _turnPID.atSetpoint();
    }

    public void resetPID()
    {
        _turnPID.reset();
    }

    public void LogTelemetry() {
        // TODO Auto-generated method stub
        SmartDashboard.putNumber("leftDriveDistance", _leftDriveEncoder.getDistance());
        SmartDashboard.putNumber("leftDriveRate", _leftDriveEncoder.getRate());
        SmartDashboard.putNumber("rightDriveDistance", _rightDriveEncoder.getDistance());
        SmartDashboard.putNumber("rightDriveRate", _rightDriveEncoder.getRate());
        SmartDashboard.putNumber("gyroHeading", getHeading());
    }

    public void ReadDashboardData() {
        // TODO Auto-generated method stub
        
    }
}
