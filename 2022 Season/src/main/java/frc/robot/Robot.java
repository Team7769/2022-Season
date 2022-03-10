// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Configuration.AutonomousMode;
import frc.robot.Configuration.Constants;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.Collector;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.ISubsystem;
import frc.robot.Subsystems.Shooter;
import frc.robot.Utilities.LEDController;
import frc.robot.Utilities.Limelight;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private XboxController _driverController;
  private XboxController _operatorController;
  private PneumaticHub _ph;
  private static Drivetrain _drivetrain;
  private static Collector _collector;
  private static Climber _climber;
  private static Shooter _shooter;
  private static Limelight _limelight;
  private static LEDController _ledController;
  private ArrayList<ISubsystem> _subsystems;

  private int _autonomousMode = AutonomousMode.kDoNothing;
  private int _autonomousCase = 0;
  private int _autonomousLoops = 0;
  private int _climbingCase = 0;
  private int _climberCurrentRung = 2;
  private int _ratchetCounter = 0;
  private boolean _climbing = false;
  private boolean _shooting = false;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    _driverController = new XboxController(Constants.kDriverControllerUsbSlot);
    _operatorController = new XboxController(Constants.kOperatorControllerUsbSlot);
    _drivetrain = Drivetrain.GetInstance();
    _limelight = Limelight.getInstance();

    _subsystems = new ArrayList<ISubsystem>();

    _subsystems.add(_drivetrain);

    if (Constants.kCompetitionRobot) {
      _collector = Collector.GetInstance();
      _shooter = Shooter.GetInstance();
      _climber = Climber.GetInstance();

      _subsystems.add(_collector);
      _subsystems.add(_climber);
      _subsystems.add(_shooter);

      _ledController = LEDController.GetInstance();
    }

    _limelight.setDashcam();
    _ph = new PneumaticHub(1);
    _ph.enableCompressorAnalog(70, 100);

    _collector.setBallCount(1);
    _drivetrain.resetGyro();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // This is the robot periodic method.
    _drivetrain.updatePose();

    _subsystems.forEach(s -> {
      s.LogTelemetry();
      s.ReadDashboardData();
    });

    if (Constants.kCompetitionRobot) {
     _shooter.zeroHood();
     _climber.resetClimbEncoder();
    }
    
    _ph.enableCompressorAnalog(70, 100);
    
    SmartDashboard.putNumber("limelightX", _limelight.getAngleToTarget());
    SmartDashboard.putBoolean("limelightValidTarget", _limelight.hasTarget());
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {

    
    _autonomousCase = 0;
    _autonomousLoops = 0;
    _drivetrain.resetGyro();
    resetOdometry();
  }
  
  public void resetOdometry()
  {
    //_drivetrain.resetOdometry();
    _drivetrain.resetEncoders();
    //_drivetrain.resetPIDControllers();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (_autonomousMode) {
      case AutonomousMode.kDoNothing:
        doNothing();
        break;
      case AutonomousMode.kTwoBallFar:
        farFourBallAuto(true);
        break;
      case AutonomousMode.kFourBallFar:
        farFourBallAuto(false);
        break;
      case AutonomousMode.kFourBallClose:
        fourBallAuto();
        break;
      default:
        // Do nothing
        doNothing();
        break;
    }
    _autonomousLoops++;
  }

  public void doNothing()
  {
    switch(_autonomousCase)
    {
      default:
        _drivetrain.tankDriveVolts(0, 0);
        break;
    }
  }
  
  public void farFourBallAuto(boolean onlyTwo)
  {
    switch(_autonomousCase)
    {
      case 0:
        _drivetrain.setFourBallFarPartOnePath();
        _drivetrain.startPath();
        _collector.intake();
        _shooter.setPukeShot();
        _shooter.readyShot();
        _autonomousCase++;
        break;
      case 1:
        _collector.intake();
        _shooter.readyShot();
        _drivetrain.followPath();

        if (_drivetrain.isPathFinished())
        {
          _autonomousLoops = 0;
          _autonomousCase++;
        }
        break;
      case 2:
        _drivetrain.tankDriveVolts(0, 0);
        _drivetrain.setFourBallFarPartTwoOutPath();
        _shooter.readyShot();
        _autonomousCase++;
        break;
      case 3:
        _collector.collectorUp();
        _collector.stopCollect();
        _shooter.readyShot();
        _limelight.setAimbot();
        
        _drivetrain.drive(0, _drivetrain.followTarget());
        if (_shooter.goShoot())
        {
          _collector.feed();
        } else {
          _collector.stopChamber();
        }

        if (_autonomousLoops >= 150)
        {
          _drivetrain.drive(0, 0);
          _shooter.stop();
          _collector.stopChamber();

          if (!onlyTwo) 
          {
            _drivetrain.startPath();
            _autonomousCase++;
          }
        }
        break;
      case 4:
        _collector.index();
        _drivetrain.followPath();
        _collector.intake();

        if (_drivetrain.isPathFinished())
        {
          _autonomousLoops = 0;
          _autonomousCase++;
        }
        break;
      case 5:
        _collector.stopChamber();
        _drivetrain.tankDriveVolts(0, 0);
        _drivetrain.setFourBallFarPartTwoBackPath();
        _autonomousCase++;
        break;
      case 6:
      _collector.index();
        _drivetrain.tankDriveVolts(0, 0);
        if (_autonomousLoops >= 50)
        {
          _autonomousLoops = 0;
          _drivetrain.startPath();
          _autonomousCase++;
        }
        break;
      case 7:
        _collector.index();
        _shooter.readyShot();
        _drivetrain.followPath();
        if (_autonomousLoops <= 50)
        {
          _collector.intake();
        }

        if (_autonomousLoops > 50) {
          _collector.stopCollect();
          _collector.collectorUp();
        }
        if (_autonomousLoops >= 100) {
          _collector.collectorDown();
        }

        if (_drivetrain.isPathFinished())
        {
          _autonomousCase++;
        }
        break;
      case 8:
        _limelight.setAimbot();
        _shooter.readyShot();
        _drivetrain.drive(0, _drivetrain.followTarget());
        if (_shooter.goShoot())
        {
          _collector.feed();
        } else {
          _collector.stopChamber();
        }
        break;
      default:
        _drivetrain.tankDriveVolts(0, 0);
        break;
    }
  }

  public void fourBallAuto()
  {
    switch(_autonomousCase)
    {
      case 0:
        _drivetrain.setDriveForwardAndShootPath();
        _drivetrain.startPath();
        _collector.intake();
        _shooter.setFarShot();
        _shooter.readyShot();
        _autonomousCase++;
        break;
      case 1:
        _collector.intake();
        _shooter.readyShot();
        _drivetrain.followPath();

        if (_drivetrain.isPathFinished())
        {
          _autonomousLoops = 0;
          _autonomousCase++;
        }
        break;
      case 2:
        _drivetrain.tankDriveVolts(0, 0);
        _drivetrain.setCollectTwoFromTerminalPath();
        _shooter.readyShot();
        _autonomousCase++;
        //_autonomousCase = 9000;
        break;
      case 3:
        _collector.collectorUp();
        _collector.stopCollect();
        _shooter.readyShot();
        _limelight.setAimbot();
        
        _drivetrain.drive(0, _drivetrain.followTarget());
        if (_shooter.goShoot())
        {
          _collector.feed();
        } else {
          _collector.stopChamber();
        }

        if (_autonomousLoops >= 150)
        {
          _shooter.stop();
          _drivetrain.startPath();
          _collector.stopChamber();
          _autonomousCase++;
        }
        break;
      case 4:
        _collector.index();
        _drivetrain.followPath();
        _collector.intake();

        if (_drivetrain.isPathFinished())
        {
          _autonomousLoops = 0;
          _autonomousCase++;
        }
        break;
      case 5:
        _collector.stopChamber();
        _drivetrain.tankDriveVolts(0, 0);
        _drivetrain.setDriveBackFromTerminalPath();
        _autonomousCase++;
        break;
      case 6:
      _collector.index();
        _drivetrain.tankDriveVolts(0, 0);
        if (_autonomousLoops >= 50)
        {
          _autonomousLoops = 0;
          _drivetrain.startPath();
          _autonomousCase++;
        }
        break;
      case 7:
        _collector.index();
        _shooter.readyShot();
        _drivetrain.followPath();
        if (_autonomousLoops <= 50)
        {
          _collector.intake();
        }

        if (_autonomousLoops > 50) {
          _collector.stopCollect();
          _collector.collectorUp();
        }
        if (_autonomousLoops >= 100) {
          _collector.collectorDown();
        }

        if (_drivetrain.isPathFinished())
        {
          _autonomousCase++;
        }
        break;
      case 8:
        _limelight.setAimbot();
        _shooter.readyShot();
        _drivetrain.drive(0, _drivetrain.followTarget());
        if (_shooter.goShoot())
        {
          _collector.feed();
        } else {
          _collector.stopChamber();
        }
        break;
      default:
        _drivetrain.tankDriveVolts(0, 0);
        break;
    }
  }

  public void fiveBallAuto()
  {
    switch (_autonomousCase)
    {
      case 0:
        _drivetrain.setFiveBallPartOnePath();
        _drivetrain.startPath();
        _autonomousCase++;
        break;
      case 1:
        _drivetrain.followPath();

        if (_drivetrain.isPathFinished())
        {
          _autonomousLoops = 0;
          _autonomousCase++;
        }
        break;
      case 2:
        _drivetrain.tankDriveVolts(0, 0);
        _drivetrain.setFiveBallPartTwoToTerminalPath();
        _autonomousCase++;
        break;
      case 3:
        _drivetrain.tankDriveVolts(0, 0);

        if (_autonomousLoops >= 100)
        {
          _drivetrain.startPath();
          _autonomousCase++;
        }
        break;
      case 4:
        _drivetrain.followPath();

        if (_drivetrain.isPathFinished())
        {
          _autonomousLoops = 0;
          _autonomousCase++;
        }
        break;
      case 5:
        _drivetrain.tankDriveVolts(0, 0);
        _drivetrain.setFiveBallPartTwoFromTerminalPath();
        _autonomousCase++;
        break;
      case 6:
        _drivetrain.tankDriveVolts(0, 0);

        if (_autonomousLoops >= 50)
        {
          _drivetrain.startPath();
          _autonomousCase++;
        }
        break;
      case 7:
        _drivetrain.followPath();

        if (_drivetrain.isPathFinished())
        {
          _autonomousCase++;
        }
        break;
      default:
        _drivetrain.tankDriveVolts(0, 0);
        break;
    }
  }
  
  
  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    _climbing = false;
    _climbingCase = 0;
    _ratchetCounter = 0;
    
    var alliance = DriverStation.getAlliance();
    if (alliance == Alliance.Blue) {
      _ledController.setLowerLED(_ledController.kBlueHeartBeat);
    } else {
      _ledController.setLowerLED(_ledController.kRedHeartBeat);
    }
    _climber.disengageRatchet();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    if (!_climbing){
      
      teleopDrive();

      if (Constants.kCompetitionRobot)
      {
        //manualClimb();
        teleopCollect();
        teleopShoot();
      }
    }
    else {
      stopAll();
      teleopClimb();
    }
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    var alliance = DriverStation.getAlliance();
    if (alliance == Alliance.Blue) {
      _ledController.setLowerLED(_ledController.kBlueHeartBeat);
    } else {
      _ledController.setLowerLED(_ledController.kRedHeartBeat);
    }
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    _limelight.setDashcam();

    _climber.resetClimbEncoder();
    _autonomousMode = (int) SmartDashboard.getNumber("autonomousMode", 0);
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  private void teleopDrive() {

    var augmentTurn = 0.0;

    var throttle = -_driverController.getLeftY();
    var turn = _driverController.getRightX();

    if (Math.abs(_driverController.getLeftTriggerAxis()) > 0.5)
    {
      _limelight.setAimbot();
      augmentTurn = _drivetrain.followTarget();
    } else {
      _limelight.setDashcam();
    }
    
    SmartDashboard.putNumber("throttle", throttle);
    SmartDashboard.putNumber("turn", turn);
    SmartDashboard.putNumber("augmentTurn", augmentTurn);
    _drivetrain.drive(throttle, (turn * .75) + augmentTurn);

    if (_driverController.getBackButton() && _driverController.getStartButton() && _climber.isExtendFinished()){
      _climbing = true;
    } else if (_driverController.getBackButton() && !_climber.isExtendFinished())
    {
      _collector.collectorUp();
      _climber.disengageRatchet();
      _climber.setClimbPosition(Constants.kClimbPullUpPosition);
      _climber.extend();
    } else if (_climber.isExtendFinished())
    {
      if (_ratchetCounter < 50) {
        _ratchetCounter++;
      } else {
        _climber.engageRatchet();
      }

      _climber.stopClimb();
    }
    
    if (_drivetrain.isTurnFinished())
      {
        SmartDashboard.putBoolean("lockedOn", true);
        //_ledController.setOnTargetState();
      } else {
        SmartDashboard.putBoolean("lockedOn", false);
        //_ledController.setTrackingTargetState();
      }
  }

  private void teleopShoot() {
    if (_operatorController.getAButtonPressed())
    {
      _shooter.setCloseShot();
    } else if (_operatorController.getBButtonPressed()) { 
      _shooter.setPukeShot();
      // Other Shot
    } else if (_operatorController.getXButtonPressed())
    {
      _shooter.setZoneShot();
    } else if (_operatorController.getYButtonPressed())
    {
      _shooter.setFarShot();
    }

    if (Math.abs(_driverController.getLeftTriggerAxis()) >= 0.5)
    {
      _shooter.readyShot();
      
      if (_driverController.getRightTriggerAxis() > 0.5 && _shooter.goShoot())  {
        _shooting = true; 
        _collector.feed();
        // Shoot
      } else {
        _shooting = false;
      }
    } else {
      _shooting = false;
      _shooter.stop();
    }
  }

  private void teleopCollect()
  {
    if (_operatorController.getLeftBumper()) {
      _collector.index(); 
      _collector.intake();
    }
    else if (_operatorController.getRightBumper()) { // Eject
      _collector.eject();
    }
    else if (!_shooting) {
      _collector.index();
      _collector.stopCollect();
      // Stop collector
    }

    if (_operatorController.getRightTriggerAxis() > 0.5) { 
      _collector.collectorUp();
    } else if (_operatorController.getLeftTriggerAxis() > 0.5) {
      _collector.collectorDown();
    }

  }

  private void logClimbState(String message)
  {
    SmartDashboard.putString("climbState", message);
  }

  private void teleopClimb()
  {
    switch (_climbingCase) {
      case 0:
        _climber.engageRatchet();
        logClimbState("Engage Ratchet");
        _climbingCase = 1;
        break;
      case 1:
        _climber.climb();

        logClimbState("Climbing");
        if (_climber.isLimitSwitchPressed())
        {
          logClimbState("Stop climbing");
          _climber.stopClimb();
          _climbingCase = 2;
        }
        break;
      case 2:
        
        logClimbState("Waiting for confirmation.");
        _ledController.setWaitingForConfirmation();
        SmartDashboard.putBoolean("driver start", _driverController.getAButton());
        if (_driverController.getAButton()){
          _ledController.setFire();
          logClimbState("Set climber foward.");
          _climber.disengageRatchet();
          _climber.setClimberForward();
          _climber.setClimbPosition(Constants.kClimbExtendedPosition);
          _ratchetCounter = 0;
          _climbingCase = 3;
        }
        break;
      case 3:
        _ratchetCounter++;

        if (_ratchetCounter >= 100)
        {
          _climbingCase = 4;
        }
        break;
      case 4:
        _climber.extend();

        if (_climber.isExtendFinished())
        {
          _climber.stopClimb();
          _ratchetCounter = 0;
          _climbingCase = 5;
        } else {
          logClimbState("Extending.");
        }

        break;
      case 5:
        _climber.stopClimb();
        _ratchetCounter++;
        if (_ratchetCounter >= 25) {
          _climber.engageRatchet();
          _climber.setClimberReverse();
          _climbingCase = 6;
        }
        break;
      case 6:
        logClimbState("Waiting for confirmation.");
        _ledController.setWaitingForConfirmation();
        if (_driverController.getAButton()){
          _ledController.setFire();
          _climbingCase = 7;
        }
        break;
      case 7:
        _climber.climb();
        
        if (_climber.isLimitSwitchPressed())
        {
          logClimbState("Stop climbing");
          _climberCurrentRung++;
          _climber.stopClimb();

          if (_climberCurrentRung >= 4)
          {
            _climbingCase = 8;
          } else {
            _climbingCase = 2;
          }
        }
        break;
      default:
        _climber.engageRatchet();
        logClimbState("Done climbing");
        break;
    }
  }
  private void manualClimb(){
    
    if (_driverController.getAButtonPressed())
    {
      _climber.engageRatchet();
    } else if (_driverController.getBButtonPressed())
    {
      _climber.disengageRatchet();
    }

    if (_driverController.getXButtonPressed())
    {
      _climber.setClimberForward();
    } else if (_driverController.getYButtonPressed())
    {
      _climber.setClimberReverse();
    }

    if (_driverController.getLeftBumper())
    {
      _climber.setClimberMotor(1.0);
    } else if (_driverController.getRightBumper())
    {
      _climber.setClimberMotor(-1.0);
    } else {
      _climber.setClimberMotor(0.0);
    }
  }

  private void stopAll() {
    _drivetrain.drive(0, 0);
    _collector.stopChamber();
    _collector.stopCollect();
    _shooter.stop();
  }
}
