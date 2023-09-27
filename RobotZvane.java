package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.hardware.bosch.BHI260IMU;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Blinker;

import org.firstinspires.ftc.robotcore.external.android.AndroidSoundPool;
import org.firstinspires.ftc.robotcore.external.android.AndroidTextToSpeech;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


public class RobotZvane {

    public DcMotorEx pewMotor;
    public DcMotorEx pewMotor2;
    public DcMotorEx motorIntake1;
    public DcMotorEx motorIntake2;
    public DcMotorEx rightMotor;
    public DcMotorEx leftMotor;
    // public DistanceSensor senzorUdaljenosti;
    public AprilTagProcessor aprilTag;
    public VisionPortal visionPortal;
    public WebcamName webCam;
    public Servo servoDizalica;
    public Servo servoPlavi;
    public Servo servoZeleni;
    public BHI260IMU imu;

    static final double     COUNTS_PER_MOTOR_REV    = 28.0;
    static final double     DRIVE_GEAR_REDUCTION    = 30.21;
    static final double     WHEEL_CIRCUMFERENCE_MM  = 90.0 * Math.PI;

    static final double     COUNTS_PER_WHEEL_REV    = COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION;

    static final double     COUNTS_PER_MM           = COUNTS_PER_WHEEL_REV / WHEEL_CIRCUMFERENCE_MM;

    public void init(HardwareMap mapaDijelova) {

        motorIntake1 = mapaDijelova.get(DcMotorEx.class, "motorIntake1");
        motorIntake2 = mapaDijelova.get(DcMotorEx.class, "motorIntake2");
        leftMotor = mapaDijelova.get(DcMotorEx.class, "leftMotor");
        rightMotor = mapaDijelova.get(DcMotorEx.class, "rightMotor");
        pewMotor = mapaDijelova.get(DcMotorEx.class, "pewMotor");
        pewMotor2 = mapaDijelova.get(DcMotorEx.class, "pewMotor2");
        // senzorUdaljenosti = mapaDijelova.get(DistanceSensor.class, "senzorUdaljenosti");
        webCam = mapaDijelova.get(WebcamName.class, "Webcam 1");
        servoPlavi = mapaDijelova.get(Servo.class, "servoPlavi");
        servoZeleni = mapaDijelova.get(Servo.class, "servoZeleni");
        imu = mapaDijelova.get(BHI260IMU.class, "imu");
        servoDizalica = mapaDijelova.get(Servo.class, "servoDizalica");

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //pewMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftMotor.setPower(0);
        rightMotor.setPower(0);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    public double CalculateInchesToMoveForward(double inches){
        return (int)(inches * 25.4 * COUNTS_PER_MM); // Inches * inchesToMM * CPmm
    }
}
