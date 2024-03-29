// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
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
import frc.robot.Utilities.VisionTargetState;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private XboxController _driverController;
  private XboxController _operatorController;
  private PowerDistribution _pd;
  private PneumaticsControlModule _pcm;
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
  private int _extendCounter = 0;
  private int _aimLoops = 0;
  private int _testDriveMode = 0;
  private boolean _finishedAiming = false;
  private boolean _climbing = false;
  private boolean _shooting = false;
  // private Timer _climbTimer = new Timer();
  // private Timer _disabledTimer = new Timer();

  // Fastest climb recorded, update as needed
  //private double _fastestClimb = 15.5;
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
    _pcm = new PneumaticsControlModule();
    _pcm.enableCompressorDigital();

    _pd = new PowerDistribution();

    _collector.setBallCount(1);
    _drivetrain.resetGyro();
    _shooter.zeroHood();

    //CameraServer.startAutomaticCapture();
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

    // _subsystems.forEach(s -> {
    //   // s.ReadDashboardData();
    //   s.LogTelemetry();
    // });
    
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

    _drivetrain.setBrakeMode();
    _ledController.setUpperLED(_ledController.color1HeartBeat);
    _autonomousCase = 0;
    _autonomousLoops = 0;
    _drivetrain.resetGyro();
    _shooter.zeroHood();
    resetOdometry();

    switch (_autonomousMode) {
      case AutonomousMode.kDoNothing:
        break;
      case AutonomousMode.kTwoBallFar:
        _drivetrain.setFourBallFarPartOnePath();
        _shooter.setTwoBallShot();
        break;
      case AutonomousMode.kTwoBallSteal:
        _drivetrain.setFourBallFarPartOnePath();
        _shooter.setTwoBallShot();
        break;
      case AutonomousMode.kFiveBallRed:
      case AutonomousMode.kFiveBallBlue:
        break;
    }
    
    // var alliance = DriverStation.getAlliance();
    // if (alliance == Alliance.Blue) {
    //   _ledController.setLowerLED(_ledController.kBlueHeartBeat);
    // } else {
    //   _ledController.setLowerLED(_ledController.kRedHeartBeat);
    // }
  }
  
  public void resetOdometry()
  {
    _drivetrain.resetEncoders();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    _drivetrain.updatePose();
    switch (_autonomousMode) {
      case AutonomousMode.kDoNothing:
        doNothing();
        break;
      case AutonomousMode.kTwoBallFar:
        twoBallStealAuto(true);
        break;
      case AutonomousMode.kTwoBallSteal:
        twoBallStealAuto(false);
        break;
      case AutonomousMode.kFiveBallRed:
        fiveBallAuto(true);
        break;
      case AutonomousMode.kFiveBallBlue:
        fiveBallAuto(false);
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

  public void twoBallStealAuto(boolean onlyTwo)
  {
    switch(_autonomousCase)
    {
      case 0:
        _collector.intake();

        if (_autonomousLoops >= 100) {
          _drivetrain.startPath();
          _autonomousCase++;
        }
        break;
      case 1:
        _collector.intake();
        _drivetrain.followPath();

        if (_drivetrain.isPathFinished())
        {
          _autonomousLoops = 0;
          _autonomousCase++;
        }
        break;
      case 2:
        if (_autonomousLoops < 1) {
          _drivetrain.setTwoBallStealPath();
        }
        _drivetrain.tankDriveVolts(0, 0);
        //_shooter.readyShot();

        if (_autonomousLoops >= 100) {
          _autonomousLoops = 0;
          _autonomousCase++;
        }
        break;
      case 3:
        _collector.collectorUp();
        _collector.stopCollect();
        _limelight.setAimbot();

        if (_autonomousLoops >= 150)
        {
          _drivetrain.drive(0, 0);
          _shooter.stop();
          _collector.stopChamber();

          if (!onlyTwo) 
          {
            _ledController.setUpperLED(_ledController.kRainbowGlitter);
            _drivetrain.startPath();
            _autonomousCase++;
          }
        } else {
          _shooter.readyShot();
          if ((_shooter.goShoot() && _drivetrain.isTurnFinished()) || _shooting) {
            _shooting = true;
            _ledController.setUpperLED(_ledController.kWaitingForConfirmation);
            _collector.feed();
          } else {
            _ledController.setUpperLED(_ledController.kYellow);
            _collector.stopChamber();
          }
          _drivetrain.drive(0, _drivetrain.followTarget());
        }
        break;
      case 4:
        _shooting = false;
        _drivetrain.followPath();
        _collector.intake();

        if (_drivetrain.isPathFinished())
        {
          _shooter.setPukeShot();
          _autonomousLoops = 0;
          _autonomousCase++;
        }
        break;
      case 5:
        _collector.stopChamber();
        _drivetrain.tankDriveVolts(0, 0);
        _shooter.readyShot();
        _autonomousCase++;
        break;
      case 6:
        _shooter.readyShot();
        _drivetrain.tankDriveVolts(0, 0);
        if (_autonomousLoops >= 50)
        {
          _collector.feed();
        } else if (_autonomousLoops >= 200) {
          _collector.stopCollect();
          _collector.stopChamber();
        }
        break;
      default:
        _drivetrain.tankDriveVolts(0, 0);
        break;
    }
  }

  // public void farFourBallAuto(boolean onlyTwo)
  // {
  //   switch(_autonomousCase)
  //   {
  //     case 0:
  //       if (_autonomousLoops < 1) {
  //         _drivetrain.setFourBallFarPartOnePath();
  //         _shooter.setTwoBallShot();
  //       }
  //       _collector.intake();

  //       if (_autonomousLoops >= 100) {
  //         _drivetrain.startPath();
  //         _autonomousCase++;
  //       }
  //       break;
  //     case 1:
  //       _collector.intake();
  //       _drivetrain.followPath();

  //       if (_drivetrain.isPathFinished())
  //       {
  //         _autonomousLoops = 0;
  //         _autonomousCase++;
  //       }
  //       break;
  //     case 2:
  //       _drivetrain.tankDriveVolts(0, 0);
  //       _drivetrain.setFourBallFarPartTwoOutPath();
  //       _shooter.readyShot();
  //       _autonomousCase++;
  //       break;
  //     case 3:
  //       _collector.collectorUp();
  //       _collector.stopCollect();
  //       _limelight.setAimbot();

  //       if (_autonomousLoops >= 150)
  //       {
  //         _drivetrain.drive(0, 0);
  //         _shooter.stop();
  //         _collector.stopChamber();

  //         if (!onlyTwo) 
  //         {
  //           _ledController.setUpperLED(_ledController.color2HeartBeat);
  //           _drivetrain.startPath();
  //           _autonomousCase++;
  //         }
  //       } else {
          
  //         if ((_shooter.goShoot() && _drivetrain.isTurnFinished()) || _shooting) {
  //           _shooting = true;
  //           _ledController.setUpperLED(_ledController.kWaitingForConfirmation);
  //           _collector.feed();
  //         } else {
  //           _ledController.setUpperLED(_ledController.kYellow);
  //           _collector.stopChamber();
  //         }
  //         _drivetrain.drive(0, _drivetrain.followTarget());
  //         _shooter.readyShot();
  //       }
  //       break;
  //     case 4:
  //       _shooting = false;
  //       _collector.index();
  //       _drivetrain.followPath();
  //       _collector.intake();

  //       if (_drivetrain.isPathFinished())
  //       {
  //         _autonomousLoops = 0;
  //         _autonomousCase++;
  //       }
  //       break;
  //     case 5:
  //       _collector.stopChamber();
  //       _drivetrain.tankDriveVolts(0, 0);
  //       _drivetrain.setFourBallFarPartTwoBackPath();
  //       _autonomousCase++;
  //       break;
  //     case 6:
  //     _collector.index();
  //       _drivetrain.tankDriveVolts(0, 0);
  //       if (_autonomousLoops >= 50)
  //       {
  //         _autonomousLoops = 0;
  //         _drivetrain.startPath();
  //         _autonomousCase++;
  //       }
  //       break;
  //     case 7:
  //       _collector.index();
  //       _shooter.readyShot();
  //       _drivetrain.followPath();
  //       if (_autonomousLoops <= 50)
  //       {
  //         _collector.intake();
  //       }

  //       if (_autonomousLoops > 50) {
  //         _collector.stopCollect();
  //         _collector.collectorUp();
  //       }
  //       if (_autonomousLoops >= 100) {
  //         _collector.collectorDown();
  //       }

  //       if (_drivetrain.isPathFinished())
  //       {
  //         _autonomousCase++;
  //       }
  //       break;
  //     case 8:
  //       _limelight.setAimbot();
  //       _shooter.readyShot();
  //       _drivetrain.drive(0, _drivetrain.followTarget());
  //       if ((_shooter.goShoot() && _drivetrain.isTurnFinished()) || _shooting)
  //       {
  //         _shooting = true;
  //         _ledController.setUpperLED(_ledController.kWaitingForConfirmation);
  //         _collector.feed();
  //       } else {
  //         _ledController.setUpperLED(_ledController.kYellow);
  //         _collector.stopChamber();
  //       }
  //       break;
  //     default:
  //       _drivetrain.tankDriveVolts(0, 0);
  //       break;
  //   }
  // }

  public void fiveBallAuto(boolean isRed)
  {
    switch(_autonomousCase)
    {
      case 0:
        if (_autonomousLoops < 1) {
          _drivetrain.setDriveForwardAndShootPath();
        }
        _collector.intake();

        if (_autonomousLoops >= 50) {
          _shooter.setAutoShot();
          _drivetrain.startPath();
          _autonomousCase++;
        }
        break;
      case 1:
        _collector.intake();
        _shooter.readyShot();
        _drivetrain.followPath();

        _limelight.setAimbot();
        if (_drivetrain.isPathFinished())
        {
          _autonomousLoops = 0;
          _autonomousCase++;
        }
        break;
      case 2:
        _drivetrain.tankDriveVolts(0, 0);

        if (isRed) {
          _drivetrain.setCollectTwoFromTerminalRedPath();
        } else {
          _drivetrain.setCollectTwoFromTerminalBluePath();
        }
        _shooter.readyShot();
        _autonomousCase++;
        _aimLoops = 0;
        _finishedAiming = false;
        _autonomousLoops = 0;
        //_autonomousCase = 9000;
        break;
      case 3:
        _shooter.readyShot();
        
        if ((_shooter.goShoot() && _finishedAiming) || _shooting)
        {
          _collector.collectorUp();
          _collector.stopCollect();
          _drivetrain.tankDriveVolts(0, 0);
          _shooting = true;
          _ledController.setUpperLED(_ledController.kWaitingForConfirmation);
          _collector.feed();
        } else {
          _drivetrain.drive(0, _drivetrain.followTarget());
          _ledController.setUpperLED(_ledController.kYellow);
          _collector.index();
        }

        if (_drivetrain.isTurnFinished() && _limelight.hasTarget()) {
          _aimLoops++;
          if (_aimLoops > 5) {
            _finishedAiming = true;
          }
        }

        if (_autonomousLoops >= 87)
        {
          _ledController.setUpperLED(_ledController.kFireMedium);
          _shooter.stop();
          _drivetrain.startPath();
          _collector.stopChamber();
          _autonomousCase++;
        }
        break;
      case 4:
        _shooting = false;
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
      _collector.intake();
        _drivetrain.tankDriveVolts(0, 0);
        if (_autonomousLoops >= 50)
        {
          _shooter.setCloseShot();
          _autonomousLoops = 0;
          _drivetrain.startPath();
          _autonomousCase++;
        }
        break;
      case 7:
        //_shooter.readyShot();
        _drivetrain.followPath();
        _collector.intake();
        // if (_autonomousLoops <= 50)
        // {
        //   _collector.intake();
        // }

        // if (_autonomousLoops > 50) {
        //   _collector.stopCollect();
        //   _collector.collectorUp();
        // }
        // if (_autonomousLoops >= 100) {
        //   _collector.collectorDown();
        // }

        if (_drivetrain.isPathFinished())
        {
          _drivetrain.setFifthBallPath();
          _autonomousLoops = 0;
          _autonomousCase++;
        }
        break;
      case 8:
         _shooter.setAutoShot();
         _limelight.setAimbot();
         _shooter.readyShot();
         _drivetrain.startPath();
         _autonomousCase++;
        // _limelight.setAimbot();
        // _shooter.readyShot();
        // if (_shooter.goShoot() || _shooting)
        // {
        //   _drivetrain.tankDriveVolts(0, 0);
        //   _shooting = true;
        //   _ledController.setWaitingForConfirmation();
        //   _collector.feed();
        // } else {
        //   _ledController.setUpperLED(_ledController.kYellow);
        //   _collector.stopChamber();
        // }

        // if (_autonomousLoops > 50) {
        //   _drivetrain.resetPID();
        //   _shooter.setZoneShot();
        //   _drivetrain.startPath();
        //   _autonomousCase++;
        // }
        break;
      case 9:
        _shooter.readyShot();
        _drivetrain.followPath();
        _collector.intake();

        if (_drivetrain.isPathFinished()) {
          _drivetrain.tankDriveVolts(0, 0);
          _shooting = false;
          _aimLoops = 0;
          _finishedAiming = false;
          _autonomousLoops = 0;
          _autonomousCase++;
        }
        break;
      case 10:
        _shooter.readyShot();
        if ((_shooter.goShoot() && _finishedAiming) || _shooting || _autonomousLoops >= 75) {
          _drivetrain.tankDriveVolts(0, 0);
          _ledController.setWaitingForConfirmation();
          _shooting = true;
          _collector.feed();
        } else {
          _drivetrain.drive(0, _drivetrain.followTarget());
          _collector.intake();
          _ledController.setUpperLED(_ledController.kYellow);

          if (_drivetrain.isTurnFinished() && _limelight.hasTarget()) {
            _aimLoops++;
            if (_aimLoops > 5) {
              _finishedAiming = true;
            }
          }
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
    _drivetrain.setCoastMode();
    _climbing = false;
    _climbingCase = 0;
    _ratchetCounter = 0;
    _ledController.setTeleopIdle();
    _ledController.setLowerLED(_ledController.kBlueHeartBeat);
    
    // var alliance = DriverStation.getAlliance();
    // if (alliance == Alliance.Blue) {
    //   _ledController.setLowerLED(_ledController.kBlueHeartBeat);
    // } else {
    //   _ledController.setLowerLED(_ledController.kRedHeartBeat);
    // }
    _climber.resetClimbEncoder(false);
    _climber.engageRatchet();
    _shooter.setAutoShot();
    _drivetrain.configTeleopTurn();
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
    
    var totalCurrent = _pd.getTotalCurrent();
    SmartDashboard.putNumber("totalCurrent", totalCurrent);

    if (totalCurrent > 220) {
      _driverController.setRumble(RumbleType.kLeftRumble, 1.0);
    } else {
      _driverController.setRumble(RumbleType.kLeftRumble, 0);
    }
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    _limelight.setAimbot();
    //_disabledTimer.start();
    // var alliance = DriverStation.getAlliance();
    // if (alliance == Alliance.Blue) {
    //   _ledController.setLowerLED(_ledController.kBlueHeartBeat);
    // } else {
    //   _ledController.setLowerLED(_ledController.kRedHeartBeat);
    // }
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    // if (_disabledTimer.get() > 2) {
    //   _drivetrain.setCoastMode();
    //   _disabledTimer.stop();
    //   _disabledTimer.reset();
    // }
      //SmartDashboard.putNumber("limelightDistance", _limelight.getDistanceToTarget());
    _autonomousMode = (int) SmartDashboard.getNumber("autonomousMode", 0);
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {

    // var slewRate = SmartDashboard.getNumber("slewRate", 0.5);
    // SmartDashboard.putNumber("slewRate", slewRate);
    // _filter = new SlewRateLimiter(slewRate);

    // _testDriveMode = 0;
    // _drivetrain.setCoastMode();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {

    // if (_driverController.getAButton()) {
    //   _testDriveMode = 0;
    //   SmartDashboard.putString("testDriveMode", "Cubic Smoothing");
    // } else if (_driverController.getBButton()) {
    //   _testDriveMode = 1;
    //   SmartDashboard.putString("testDriveMode", "Slew Rate Filter");
    // } else if (_driverController.getXButton()) {
    //   _testDriveMode = 2;
    //   SmartDashboard.putString("testDriveMode", "Curvature Drive");
    // } else if (_driverController.getYButton()) {
    //   SmartDashboard.putString("testDriveMode", "Direct Drive");
    //   _testDriveMode = 3;
    // } else if (_driverController.getStartButton()) {
    //   _testDriveMode = 4;
    //   SmartDashboard.putString("testDriveMode", "Current Drive");
    // }

    // var throttle = -_driverController.getLeftY();
    // var turn = _driverController.getRightX();

    // if (Math.abs(throttle) <= 0.1) {
    //   throttle = 0;
    // }
    // if (Math.abs(turn) <= 0.1) {
    //   turn = 0;
    // }

    // switch(_testDriveMode) {
    //   case 0:

    //     // Smoothing algorithm for x^3
    //     if (throttle > 0.0)
    //       throttle = (1 - 0.1) * Math.pow(throttle, 3) + 0.1;
    //     else if (throttle < 0.0)
    //       throttle = (1 - 0.1) * Math.pow(throttle, 3) - 0.1;

    //     // Smoothing algorithm for x^3
    //     if (turn > 0.0)
    //       turn = (1 - 0.1) * Math.pow(turn, 3) + 0.1;
    //     else if (turn < 0.0)
    //       turn = (1 - 0.1) * Math.pow(turn, 3) - 0.1;

    //     _drivetrain.arcadeDrive(throttle, turn);
    //     _ledController.setNewRecord();
    //     break;
    //   case 1:
    //     _drivetrain.arcadeDrive(_filter.calculate(throttle), turn);
    //     _ledController.setUpperLED(_ledController.bpmParty);
    
    //     break;
    //   case 2:
    //     var quickTurn = _driverController.getLeftBumper();
    //     _drivetrain.curvatureDrive(throttle, turn, quickTurn);
    //     _ledController.setUpperLED(_ledController.kGoldStrobe);
    
    //     break;
    //   case 3:
    //     _drivetrain.directDrive(throttle, turn);
    //     _ledController.setUpperLED(_ledController.kBlueShot);
    //     break;
    //   default:
    //     _drivetrain.drive(throttle, turn);
    //     _ledController.setUpperLED(_ledController.kConfetti);
    //     break;
    // }

    // SmartDashboard.putNumber("driverY", _driverController.getLeftY());
    // SmartDashboard.putNumber("driverX", _driverController.getRightX());
  }

  private void teleopDrive() {

    var augmentTurn = 0.0;

    var throttle = -_driverController.getLeftY();
    var turn = _driverController.getRightX();

    if (Math.abs(_driverController.getLeftTriggerAxis()) > 0.5)
    {
      _limelight.setAimbot();

      if (!_finishedAiming){
        augmentTurn = _drivetrain.followTarget();
      } else {
        _drivetrain.tankDriveVolts(0, 0);
      }

      if (_drivetrain.isTurnFinished() && _limelight.hasTarget()) {
        _aimLoops++;
        if (_aimLoops > 5) {
          _finishedAiming = true;
        }
      }
    } else {
      _aimLoops = 0;
      _drivetrain.resetTargetAngle();
      _drivetrain.resetPID();
      _finishedAiming = false;
      _limelight.setDashcam();
    }
    
    _drivetrain.drive(throttle, (turn * .75) + augmentTurn);

    if (_driverController.getBackButton() && _driverController.getStartButton() && _climber.isExtendFinished()){
      _climbing = true;
    } else if (_driverController.getBackButton() && !_climber.isExtendFinished())
    {
      _collector.collectorUp();
      _climber.disengageRatchet();
      _climber.setClimbPosition(Constants.kClimbPullUpPosition);

      if (_ratchetCounter < 25) {
        _ratchetCounter++;
      } else {
        _climber.extend();
      }
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
      } else {
        SmartDashboard.putBoolean("lockedOn", false);
      }
  }

  private void teleopShoot() {
    if (_operatorController.getAButtonPressed() || _driverController.getAButtonPressed())
    {
      _shooter.setAutoShot();
    } else if (_driverController.getBButtonPressed()) { 
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
      _pcm.disableCompressor();
      _shooter.readyShot();
          
        if (_driverController.getRightTriggerAxis() > 0.5)  {
          _shooting = true; 
          _collector.feed();
          // Shoot
        } else {
          _shooting = false;
        }
  
        if (_shooter.goShoot() && _finishedAiming){
          _ledController.setUpperLED(_ledController.kWaitingForConfirmation);
        } else {
          _ledController.setUpperLED(_ledController.kYellow);
        }
    } else {
      _pcm.enableCompressorDigital();
      _ledController.setTeleopIdle();
      _shooting = false;

      if (_driverController.getYButton()) {
        _shooter.resetHoodEncoder();
      } else if (_driverController.getXButton()) {
        _shooter.moveHoodDown();
      } else {
        _shooter.stop();
      }
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
      _collector.collectorUp();
      _collector.stopCollect();
      // Stop collector
    }

    // if (_operatorController.getRightTriggerAxis() > 0.5) { 
    //   _collector.collectorUp();
    // } else if (_operatorController.getLeftTriggerAxis() > 0.5) {
    //   _collector.collectorDown();
    // }

  }

  private void logClimbState(String message)
  {
    SmartDashboard.putString("climbState", message);
  }

  private void teleopClimb()
  {
    _climber.resetClimbEncoder(true);
    SmartDashboard.putNumber("climbingCase", _climbingCase);
    SmartDashboard.putNumber("currentRungNumber", _climberCurrentRung);
    //SmartDashboard.putNumber("climbTime", _climbTimer.get());
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
          _climber.setClimberForward();
          logClimbState("Stop climbing");
          _climber.stopClimb();
          _climbingCase = 2;
        }
        break;
      case 2:
        
        logClimbState("Waiting for confirmation.");
        _ledController.setWaitingForConfirmation();
        if (_driverController.getAButton()){
          _ledController.setFire();
          logClimbState("Set climber foward.");
          _climber.setClimbPosition(Constants.kClimbExtendedPosition);
          _ratchetCounter = 0;
          _climbingCase = 3;
        }
        break;
      case 3:
        _ratchetCounter++;

        if (_ratchetCounter >= 50)
        {
          _extendCounter = 0;
          _climber.stopClimb();

          if (_climberCurrentRung > 2) {
            _climbingCase = 4;
          } else {
            _climbingCase = 5;
          }
        } else if (_ratchetCounter >= 17) {
          _climber.stopClimb();
        } else if (_ratchetCounter <= 5) {
          _climber.disengageRatchet();
          _climber.climb();
        }
        break;
      case 4:
        if (_extendCounter <= 25) {
          logClimbState("Extending.");
          _climber.extend();
          _extendCounter++;
        } else {
          _climber.stopClimb();
          logClimbState("Waiting for confirmation.");
          _ledController.setWaitingForConfirmation();
          if (_driverController.getAButton()){
            _ledController.setFire();
            _climbingCase = 5;
          }
        }
        break;
      case 5:
        _climber.extend();
        if (_climber.isExtendFinished())
        {
          _climber.stopClimb();
          _ratchetCounter = 0;
          _climbingCase = 6;
        } else {
          logClimbState("Extending.");
        }
        break;
      case 6:
        _climber.stopClimb();
        _ratchetCounter++;
        if (_ratchetCounter >= 25) {
          _climber.engageRatchet();
          _climber.setClimberReverse();
          _climbingCase = 7;
        }
        break;
      case 7:
        logClimbState("Waiting for confirmation.");
        _ledController.setWaitingForConfirmation();
        if (_driverController.getAButton()){
          _ledController.setFire();
          _ratchetCounter = 0;
          _climbingCase = 8;
        }
        break;
      case 8:
      if (_ratchetCounter <= 25) {
        _climber.climb();
        _ratchetCounter++;
      } else {
        _climber.stopClimb();
        logClimbState("Waiting for confirmation.");

        if (_climberCurrentRung >= 3) {
          _ledController.setNewRecord();
        } else {
          _ledController.setWaitingForConfirmation();
        }
        if (_driverController.getAButton()){
          _ledController.setFire();
          _ratchetCounter = 0;
          _climbingCase = 9;
        }
      }
        break;
      case 9:
        _climber.climb();
        
        if (_climber.isLimitSwitchPressed())
        {
          _climber.setClimberForward();
          logClimbState("Stop climbing");
          _climberCurrentRung++;
          _climber.stopClimb();

          if (_climberCurrentRung >= 4)
          {
            _climbingCase = 10;
          } else {
            _climbingCase = 2;
          }
        } else {
          _ratchetCounter++;
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

  private double deadband(double input, double minValue) {
    if (Math.abs(input) <= minValue) {
      return 0.0;
    }

    return input;
  }
}
