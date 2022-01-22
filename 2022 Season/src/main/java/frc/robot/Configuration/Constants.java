package frc.robot.Configuration;

public final class Constants {

    // Usb Slots
    public static final int kDriverControllerUsbSlot = 0;
    public static final int kOperatorControllerUsbSlot = 1;
    
    // CAN Device Ids
    public static final int kLeftFrontDriveDeviceId = 2;
    public static final int kLeftRearDriveDeviceId = 3;
    public static final int kRightFrontDriveDeviceId = 4;
    public static final int kRightRearDriveDeviceId = 5;
    
    //DIO
    public static final int kLeftEncoderPortA = 0;
    public static final int kLeftEncoderPortB = 1;
    public static final int kRightEncoderPortA = 2;
    public static final int kRightEncoderPortB = 3;

    // Test Robot Configuration

    public static final int kDriveEncoderTicksPerRevolution = 2048;
    public static final double kDriveWheelDiameter = 0.15;
    public static final double kDriveDistancePerPulse = (kDriveWheelDiameter * Math.PI) / kDriveEncoderTicksPerRevolution;
}
