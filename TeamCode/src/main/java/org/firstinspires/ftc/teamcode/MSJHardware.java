package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorColor;
import android.app.Activity;
import android.graphics.Color;
import android.view.View;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

public class MSJHardware {

        //Create Drivetrain Motors
        public DcMotor frontRightMotor = null;
        public DcMotor frontLeftMotor = null;
        public DcMotor backRightMotor = null;
        public DcMotor backLeftMotor = null;

        public DcMotor spinnerMotor = null;

        public ColorSensor colorSensor;

        public Servo dropperServo = null;
        public Servo armServo = null;


        //Create Shooter Motor
    /*    public DcMotor shooterMotor = null;

        //Create Lift
        public DcMotor liftMotor = null;

        //Create Servos
        public Servo dropperServo = null;
        public CRServo loaderServo = null;
        public Servo clawServo = null;
*/
        //Intake system motor
        public DcMotor intakeMotor = null;

        //Arm motor
     public DcMotor armMotor = null;

        //Additional Variables
        HardwareMap hwMap = null;
        public ElapsedTime runtime = new ElapsedTime();
        public MSJHardware(){

        }
        // public MSJHardware(HardwareMap ahwMap) {

        //    initialize(ahwMap);
        // }

        public void init(HardwareMap ahwMap){
            hwMap = ahwMap;

            //Connect Drive Train Motors
            frontRightMotor = hwMap.get(DcMotor.class, "frontRightMotor");
            frontLeftMotor = hwMap.get(DcMotor.class,"frontLeftMotor");
            backRightMotor = hwMap.get(DcMotor.class,"backRightMotor");
            backLeftMotor = hwMap.get(DcMotor.class,"backLeftMotor");

            spinnerMotor = hwMap.get(DcMotor.class, "spinnerMotor");
            dropperServo = hwMap.get(Servo.class,"dropperServo");
            armServo = hwMap.get(Servo.class,"armServo");

       /*     //Connect Shooter Motor
            shooterMotor = hwMap.get(DcMotor.class,"shooterMotor");

            //Connect Lift Motor
            liftMotor = hwMap.get(DcMotor.class,"liftMotor");
            //Connect Servos

            loaderServo = hwMap.get(CRServo.class, "loaderServo");
            clawServo = hwMap.get(Servo.class,"clawServo");
*/
            //Connect intake Motors
            intakeMotor = hwMap.get(DcMotor.class, "intakeMotor");

           armMotor = hwMap.get(DcMotor.class, "armMotor");
            colorSensor = hwMap.get(ColorSensor.class, "colorSensor");


            //Set Up Motor Direction
            frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
            frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
            backRightMotor.setDirection(DcMotor.Direction.REVERSE);
            backLeftMotor.setDirection(DcMotor.Direction.FORWARD);

            spinnerMotor.setDirection(DcMotor.Direction.FORWARD);

/*
            shooterMotor.setDirection(DcMotor.Direction.REVERSE);

            liftMotor.setDirection(DcMotor.Direction.REVERSE);

*/
          armMotor.setDirection(DcMotor.Direction.REVERSE);

            intakeMotor.setDirection(DcMotor.Direction.FORWARD);
            //Set Motor Mode  (For now we will run it without an encoder, but when we do stop_and_reset_encoder for each motor and then run_using_encoder for each motor)
            // We should be using encoders b/c running based off time is not reliable b/c it depends on charge of battery and the power the motors are receiving
            frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            spinnerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
/*
            liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //  liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

*/
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            //Set ZERO POWER BEHAVIOR for Drive Train as BRAKE so that the motors stop turning
            frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            spinnerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

/*
            liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

*/
             armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


            //Set ZERO POWER BEHAVIOR for Shooters as FREESPIN so that the motor continue to spin freely
//            shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //Set Motors to use no power
            frontRightMotor.setPower(0);
            frontLeftMotor.setPower(0);
            backRightMotor.setPower(0);
            backLeftMotor.setPower(0);

            spinnerMotor.setPower(0);

         /*
            shooterMotor.setPower(0);
            liftMotor.setPower(0);
*/
            armMotor.setPower(0);


            intakeMotor.setPower(0);

            dropperServo.setPosition(0.3);
    }
}
