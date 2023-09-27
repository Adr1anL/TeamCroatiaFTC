package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.lang.Math.*;

import java.util.List;

@TeleOp(name = "Vozi Vozi", group = "PushBot")

public class VoziVozi extends LinearOpMode {


    RobotZvane robot = new RobotZvane();

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        // Start AprilTag Processor
        AprilTagProcessor tagProcessor = initAprilTagVisionPortal();
        YawPitchRollAngles robotOrientation;

        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();


        boolean inverted = false;
        double MAX_BRZINA = 0.75;
        double left = 0;
        double right = 0;
        double triggerDeadzone = 0.5;
        double servoStart = 0.5;
        double MAX_PEW = 1;
        double MIN_PEW = 0.2;
        double pew_increment = 0.1;
        double pew = 0;
        double drive;
        double turn;
        double max;
        int intakeSpeed = 1000;
        int buttonDelay = 100;

        waitForStart();

        while (opModeIsActive()) {
            robotOrientation = InitializeIMU();
            double Yaw   = robotOrientation.getYaw(AngleUnit.DEGREES);
            double Pitch = robotOrientation.getPitch(AngleUnit.DEGREES);
            double Roll  = robotOrientation.getRoll(AngleUnit.DEGREES);

            // robot.servoPlavi.setPosition(servoStart);
            // robot.servoZeleni.setPosition(servoStart);
            robot.servoDizalica.setPosition(1);

            // Basic drive functions
            drive = gamepad2.left_stick_y;
            turn = -gamepad2.right_stick_x;

            // Invert drive controls
            if (!inverted) {
                left = drive + turn;
                right = drive - turn;
            }
            if (inverted) {
                left = -drive + turn;
                right = -drive - turn;
            }

            left = left * MAX_BRZINA;
            right = right * MAX_BRZINA;

            // Normalize the values so neither exceed +/- 1.0
            max = Math.max(Math.abs(left), Math.abs(right));
            if (max > MAX_BRZINA) {
                left /= max;
                right /= max;
            }
            // Output the safe vales to the motor drive
            robot.leftMotor.setPower(left);
            robot.rightMotor.setPower(right);

            // Start motors for intake
            if (gamepad2.right_trigger > triggerDeadzone) {
                robot.motorIntake1.setVelocity(-intakeSpeed);
                robot.motorIntake2.setVelocity(-intakeSpeed);
            }

            // Stop motors for intake
            if (gamepad2.left_trigger > triggerDeadzone) {
                robot.motorIntake1.setVelocity(0);
                robot.motorIntake2.setVelocity(0);
            }

            // Invert drive controls
            if (gamepad2.x) {
                inverted = !inverted;
            }
            if (gamepad2.square) {
                robot.servoDizalica.setPosition(0);
                sleep(1000);
            }

            // Increase shooter motor RPM
            if (gamepad1.dpad_up) {
                if (pew < MAX_PEW - pew_increment) {
                    pew += pew_increment;
                    if (pew < MIN_PEW) {
                        pew += pew_increment;
                    }
                }
                robot.pewMotor.setPower(-pew);
                robot.pewMotor2.setPower(pew);
                sleep(buttonDelay);
            }

            // Increase shooter motor RPM for half increment value
            if (gamepad1.dpad_right) {
                pew += pew_increment / 2;
                robot.pewMotor.setPower(-pew);
                robot.pewMotor2.setPower(pew);
                sleep(buttonDelay);
            }

            // Decrease shooter motor RPM
            if (gamepad1.dpad_down) {
                if (pew > pew_increment) {
                    pew -= pew_increment;
                }
                robot.pewMotor.setPower(-pew);
                robot.pewMotor2.setPower(pew);
                sleep(buttonDelay);
            }

            // Start servo for blue balls
            if (gamepad1.circle) {
                robot.servoPlavi.setPosition(0);
                sleep(1000);
            }

            // Start servo for green balls
            if (gamepad1.y) {
                robot.servoZeleni.setPosition(1);
                sleep(1000);
            }
            // Stop all shooter motors
            if (gamepad1.square) {
                robot.pewMotor.setPower(0);
                robot.pewMotor2.setPower(0);
                pew = 0;
            }

            // Telemetry
            telemetry.addData("Inverted: ", inverted);
            telemetry.addData("Pew Power: ", pew);
            telemetry.addLine(String.format("PRY %3.0f %3.0f %3.0f (deg)", Pitch, Roll, Yaw));
            // telemetry.addData("Distance: ", robot.senzorUdaljenosti.getDistance(DistanceUnit.CM));

            AprilTagDetection detection = AprilTagTelemetry(tagProcessor);

            // AprilTag scanned and ready to shoot
            if (gamepad2.y && detection.metadata != null){
                try{
                    AutonomousShooting(detection, robotOrientation);
                }catch(Exception AprilTagMetadataError){
                    telemetry.addLine(String.format("AprilTagMetadataError Try again!"));
                    telemetry.update();
                }
            }

            telemetry.update();
        }
    }

    // Method to start AprilTag Processor and Vision Portal
    private AprilTagProcessor initAprilTagVisionPortal() {
        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setLensIntrinsics(1464.54923917, 1464.54923917, 719.92587377, 313.776126916)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(robot.webCam)
                .setCameraResolution(new Size(1280, 720))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .build();

        return tagProcessor;
    }

    // Method to start telemetry for Scanning April Tags
    private AprilTagDetection AprilTagTelemetry(AprilTagProcessor tagProcessor) {
        List<AprilTagDetection> currentDetections = tagProcessor.getDetections();
        telemetry.addData("AprilTags Detected: ", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            try {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                return detection;
            } catch (Exception e) {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop
        return null;
    }
    private void AutonomousShooting(AprilTagDetection detection, YawPitchRollAngles robotOrientation){
        double d = detection.ftcPose.y;
        double betaCrtano = 90 - detection.ftcPose.yaw;;
        double alfaCrtano = 90 - detection.ftcPose.pitch;
        double a = d * Math.sin(alfaCrtano);
        double b = d * Math.cos(alfaCrtano);
        double Forward;
        double Yaw;

        do{
            Yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
            telemetry.addData("Yaw: ", Yaw);
            telemetry.update();
            robot.rightMotor.setPower(0.3);
        }while(Yaw > betaCrtano && opModeIsActive());

        robot.rightMotor.setPower(0);

        double TPS = (175/ 60) * robot.COUNTS_PER_WHEEL_REV;
        Forward = robot.CalculateInchesToMoveForward(a);

        telemetry.addData("FORWARD: ", Forward);

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.leftMotor.setVelocity(TPS);
        robot.rightMotor.setVelocity(TPS);

        robot.leftMotor.setTargetPosition((int) Forward);
        robot.rightMotor.setTargetPosition((int) Forward);

        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addLine(String.format("Done."));
        telemetry.update();

    }

    private YawPitchRollAngles InitializeIMU() {
        IMU.Parameters myImuParameters;
        YawPitchRollAngles robotOrientation = null;

        myImuParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );

        try {
            robot.imu.initialize(myImuParameters);
            robotOrientation = robot.imu.getRobotYawPitchRollAngles();

        } catch (Exception imu) {
            telemetry.addLine(String.format("Failed initializing IMU!"));
        }
        return robotOrientation;
    }
}
