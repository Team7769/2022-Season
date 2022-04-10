package frc.robot.Configuration;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

public final class Constants {
    // Limelight Constants
    public static final int kLEDOff = 1;
    public static final int kLEDOn = 3;
    public static final int kCamModeVisionProcessing = 0;
    public static final int kCamModeDriver = 1;
    public static final String kTableName = "limelight";
    public static final String kValidTargetKey = "tv";
    public static final String kLEDModeKey = "ledMode";
    public static final String kCamModeKey = "camMode";
    public static final String kTargetAngleXKey = "tx";
    public static final String kTargetAngleYKey = "ty";
    
    // Usb Slots
    public static final int kDriverControllerUsbSlot = 0;
    public static final int kOperatorControllerUsbSlot = 1;
    
    // CAN Device Ids
    public static final int kLeftFrontDriveDeviceId = 2;
    public static final int kLeftMiddleDriveDeviceId = 3;
    public static final int kLeftRearDriveDeviceId = 4;
    public static final int kRightFrontDriveDeviceId = 5;
    public static final int kRightMiddleDriveDeviceId = 6;
    public static final int kRightRearDriveDeviceId = 7;
    public static final int kLeftClimbMotorDeviceId = 8;
    public static final int kRightClimbMotorDeviceId = 9;
    public static final int kHoodMotorDeviceId = 10; 
    public static final int kChamberFrontMotorDeviceId = 11;
    public static final int kChamberBackMotorDeviceId = 12;
    public static final int kCollectMotorDeviceId = 13;
    
    // Talon FX Device Ids
    public static final int kLeftShooterMotorDeviceId = 15; 
    public static final int kRightShooterMotorDeviceId= 14;

    // Solenoids
    public static final int kCollectorSolenoidChannel = 5;
    public static final int kClimberSolenoidForwardChannel = 4;
    public static final int kClimberSolenoidReverseChannel = 0;
    public static final int kRatchetSolenoidForwardChannel = 7;
    public static final int kRatchetSolenoidReverseChannel = 6;
    
    //DIO
    public static final int kLeftEncoderPortA = 0;
    public static final int kLeftEncoderPortB = 1;
    public static final int kRightEncoderPortA = 2;
    public static final int kRightEncoderPortB = 3;
    public static final int kHoodEncoderPort = 4;
    public static final int kClimbEncoderPort = 5;
    public static final int kChamberBottomPort = 6;
    public static final int kChamberTopPort = 7;

    // Left Shooter Motor Config Values
    public static final double kShooterKp= .015;
    public static final double kShooterKf= .0485;

    // Hood Target Points
    public static final double kPukeShotValue = 0.25;
    public static final double kCloseShotValue = .1;
    public static final double kZoneShotValue = 0.14;
    public static final double kFarShotValue = 0.15;

    public static final double kPukeShotSpeed = 5000;
    public static final double kCloseShotSpeed = 11500;
    public static final double kZoneShotSpeed = 13600;
    public static final double kFarShotSpeed = 14000;
    
    // Drive Forward and Shoot Values
    public static final double kDriveForwardStartX= 6.71;
    public static final double kDriveForwardStartY= 2.440;
    public static final double kDriveForwardStartRotation= -2.437994417066567;
    public static final double kDriveForwardMidX = 5.874;
    public static final double kDriveForwardMidY= 2.135;
    public static final double kDriveForwardEndX= 5.019;
    public static final double kDriveForwardEndY= 1.910;
    public static final double kDriveForwardEndRotation= -2.9;

    // Collect Two From Terminal Values
    public static final double kCollectTwoWayPointX = 3.748;
    public static final double kCollectTwoWayPointY = 2.168;
    public static final double kCollectTwoEndPointX= 1.667;
    public static final double kCollectTwoEndPointY= 2.358;
    public static final double kCollectTwoEndRotation= -2.65;
   
    // Drive Back From Terminal Values
    public static final double kDriveBackWayPointX= 4.85;
    public static final double kDriveBackWayPointY= 1.359;
    public static final double kDriveBackEndPointX= 7.639;
    public static final double kDriveBackEndPointY= 1.550;
    public static final double kDriveBackEndPointRotation= -2.1;

    // Fifth Ball Values
    public static final double kFifthBallWayPointX= 7.448;
    public static final double kFifthBallWayPointY= 0.853;
    public static final double kFifthBallEndPointX= 7.35;
    public static final double kFifthBallEndPointY= 0.30;
    public static final double kFifthBallEndPointRotation= -1.5707963267948994;
    
    //Five Ball Part One Values
    public static final double kFiveBallPartOneStartX= 8.663;
    public static final double kFiveBallPartOneStartY= 1.730;
    public static final double kFiveBallPartOneStartRotation= -1.1722738811284743;
   
    public static final double kFiveBallPartOneFirstX= 7.707; 
    public static final double kFiveBallPartOneFirstY= 0.054; 
    public static final double kFiveBallPartOneSecondX= 6.886;
    public static final double kFiveBallPartOneSecondY= 0.740;
    
    public static final double kFiveBallPartOneEndX= 5.064; 
    public static final double kFiveBallPartOneEndY= 0.864; 
    public static final double kFiveBallPartOneEndRotation= -2.4346845928296763;
    
    //Five Ball Part Two To Terminal Values
    public static final double kFiveBallPartTwoToTerminalStartX= 5.818;
    public static final double kFiveBallPartTwoToTerminalStartY= 0.864;

    public static final double kFiveBallPartTwoToTerminalEndX= 1.173;
    public static final double kFiveBallPartTwoToTerminalEndY= 2; 
    public static final double kFiveBallPartTwoToTerminalEndRotation= -2.442932828868268;
    // Five Ball Part Two From Terminal Values
    public static final double kFiveBallPartTwoFromTerminalStartX= 1.173; 
    public static final double kFiveBallPartTwoFromTerminalStartY= 2;
    
    public static final double kFiveBallPartTwoFromTerminalEndX= 5.975;
    public static final double kFiveBallPartTwoFromTerminalEndY= 1.156;
    public static final double kFiveBallPartTwoFromTerminalEndRotation= -2.5878351309802463;

    // Four Ball Far Part One Values
    public static final double kFourBallFarPartOneStartX= 5.705;
    public static final double kFourBallFarPartOneStartY= 4.912; 
    public static final double kFourBallFarPartOneStartRotation= 2.3334711293507033;

    public static final double kFourBallFarPartOneMidX= 5.315; 
    public static final double kFourBallFarPartOneMidY= 5.862; 
    
    public static final double kFourBallFarPartOneEndX= 4.85;
    public static final double kFourBallFarPartOneEndY= 6.622;
    public static final double kFourBallFarPartOneEndRotation= 2.2565258377011546;

    // Four Ball Far Part Two Values    
    public static final double kFourBallFarPartTwoStartX= 4.85;
    public static final double kFourBallFarPartTwoStartY= 6.622;

    public static final double kFourBallFarPartTwoMidX= 3.977; 
    public static final double kFourBallFarPartTwoMidY= 5.478; 
    
    public static final double kFourBallFarPartTwoEndPointX= 0.802;
    public static final double kFourBallFarPartTwoEndPointY= 2.854;
    public static final double kFourBallFarPartTwoEndRotation= -2.5712743268771217;
    
    // Four Ball Far Part Two Back Values
    public static final double kFourBallFarPartTwoBackStartX= 0.678;
    public static final double kFourBallFarPartTwoBackStartY= 2.742;

    public static final double kFourBallFarPartTwoBackMidX= 3.175; 
    public static final double kFourBallFarPartTwoBackMidY= 2.202; 
    
    public static final double kFourBallFarPartTwoBackEndPointX= 4.918;
    public static final double kFourBallFarPartTwoBackEndPointY= 2.697;
    public static final double kFourBallFarPartTwoBackEndRotation= -2.7716865861332294;

    // Two Ball Steal Values
    public static final double kTwoBallStealMidpointAX = 5.019;
    public static final double kTwoBallStealMidpointAY = 7.274;
    public static final double kTwoBallStealMidpointBX = 5.604;
    public static final double kTwoBallStealMidpointBY = 7.330;
    public static final double kTwoBallStealEndpointX = 6.45;
    public static final double kTwoBallStealEndpointY = 7.330;
    public static final double kTwoBallStealEndRotation = 0;

    //Hood PID Values
    public static final double kHoodKp= 16.0; 
    public static final double kHoodKi= 0;
    public static final double kHoodKd= 0;

    //Climber PID Values
    public static final double kClimbKp = 2;
    public static final double kClimbKi = 0;
    public static final double kClimbD = 0;

    public static final double kClimbPullUpPosition = 6.1;
    public static final double kClimbExtendedPosition = 7.75;
    //public static final double kClimbF = 0.35;
    
    // Test Robot
    // Test Robot Configuration
    // public static final boolean kCompetitionRobot = false;
    // public static final int kDriveEncoderTicksPerRevolution = 2048;
    // public static final double kDriveWheelDiameter = 0.15;
    // public static final double kDriveDistancePerPulse = (kDriveWheelDiameter * Math.PI) / kDriveEncoderTicksPerRevolution;
    
    // //Drive Characteristics
    // public static final double ksVolts = 0.149;
    // public static final double kvVoltSecondsPerMeter = 2.86;
    // public static final double kaVoltSecondsSquaredPerMeter = 0.0;
    // public static final double kTrackwidthMeters = 1.4;
    // public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);

    // public static final double kMaxSpeedMetersPerSecond = 1.5;
    // public static final double kMaxAccelerationMetersPerSecondSquared = 1.5;

    // //Path Following
    // public static final double kPathFollowingkP = 4.5;

    // // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    // public static final double kRamseteB = 2;
    // public static final double kRamseteZeta = 0.7;


    // Competition Robot
    public static final boolean kCompetitionRobot = true;
    // Robot Configuration
    public static final int kDriveEncoderTicksPerRevolution = 2048;
    public static final double kDriveWheelDiameter = 0.10;
    public static final double kDriveDistancePerPulse = (kDriveWheelDiameter * Math.PI) / kDriveEncoderTicksPerRevolution;
    
    //Drive Characteristics
    // public static final double ksVolts = 0.7201;
    // public static final double kvVoltSecondsPerMeter = 10.433;
    // public static final double kaVoltSecondsSquaredPerMeter = 0.0;
    public static final double ksVolts = 0.149;
    public static final double kvVoltSecondsPerMeter = 2.86;
    public static final double kaVoltSecondsSquaredPerMeter = 0.0;
    public static final double kTrackwidthMeters = .715;
    //public static final double kTrackwidthMeters = .72;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);

    public static final double kMaxSpeedMetersPerSecond = 3.25;
    public static final double kMaxAccelerationMetersPerSecondSquared = 2.5;

    //Path Following
    public static final double kPathFollowingkP = 4.5;

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
}
