package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import android.app.Activity;
import android.graphics.Color;
import android.view.View;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


        /**
         * This file illustrates the concept of driving a path based on encoder counts.
         * It uses the common Pushbot hardware class to define the drive on the robot.
         * The code is structured as a LinearOpMode
         *
         * The code REQUIRES that you DO have encoders on the wheels,
         *   otherwise you would use: PushbotAutoDriveByTime;
         *
         *  This code ALSO requires that the drive Motors have been configured such that a positive
         *  power command moves them forwards, and causes the encoders to count UP.
         *
         *   The desired path in this example is:
         *   - Drive forward for 19 inches
         *   - Spin right for 12 Inches
         *   - Drive Backwards for 23 inches
         *   - Stop and close the claw.
         *   - Drive forward for 47 inches
         *
         *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
         *  that performs the actual movement.
         *  This methods assumes that each movement is relative to the last stopping place.
         *  There are other ways to perform encoder based moves, but this method is probably the simplest.
         *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
         *
         * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
         * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
         */

        @Autonomous(name="Blue Outside 1", group="Autonomous")
       //   @Disabled
        public class BlueOutside1 extends LinearOpMode {

            /* Declare OpMode members. */

            MSJHardware robot = new MSJHardware(); // Use our hardware

            private ElapsedTime runtime = new ElapsedTime();

            static final double COUNTS_PER_MOTOR_REV = 537.6;    // eg: goBILDA Motor Encoder
            static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
            static final double WHEEL_DIAMETER_INCHES = 3.0;     // For figuring circumference
            static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                    (WHEEL_DIAMETER_INCHES * 3.1415);
            static final double DRIVE_SPEED = 0.5;
            static final double TURN_SPEED = 0.25;
            public double duckDistance = 3;
            public double sHubDistance = 26;
            public int armHeight = 650;
            //  @Override
            NormalizedColorSensor colorSensor;

            public void runOpMode() {

                /*
                 * Initialize the drive system variables.
                 * The init() method of the hardware class does all the work here
                 */
                robot.init(hardwareMap);

                // Send telemetry message to signify robot waiting;
                telemetry.addData("Status", "Resetting Encoders");    //
                telemetry.update();


                robot.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                // robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                // robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


                // Send telemetry message to indicate successful Encoder reset
                telemetry.addData("Path0", "Starting at %7d :%7d",
                        robot.frontLeftMotor.getCurrentPosition(),
                        robot.frontRightMotor.getCurrentPosition(),
                        robot.backLeftMotor.getCurrentPosition(),
                        robot.backRightMotor.getCurrentPosition()
                );
                telemetry.update();

                // Wait for the game to start (driver presses PLAY)
                waitForStart();

                // Step through each leg of the path,
                // Note: Reverse movement is obtained by setting a negative distance (not speed)


                telemetry.addData("Red:", robot.colorSensor.red());
                telemetry.addData("Blue:", robot.colorSensor.blue());
                telemetry.addData("Green:", robot.colorSensor.green());

                /*
        robot.liftMotor.setTargetPosition(-6650);
        robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftMotor.setPower(.75);
        while (opModeIsActive() && robot.liftMotor.isBusy() ) {
            // Display it for the driver.
            telemetry.addData("Path1",  "Running to %7d", -6650);
            telemetry.addData("Path2",  "Running at %7d",
                    robot.liftMotor.getCurrentPosition());
            telemetry.update();
        }
        robot.liftMotor.setPower(0);
*/
                // Drive forwards to get off wall
                encoderDrive(DRIVE_SPEED, 2, 2, 4.0);
                //drop intake system
                robot.dropperServo.setPosition(.74);
                //Strafe right to carousel
                encoderStrafe((DRIVE_SPEED - .25), 6, -6, 5.0);
                //turn towards carousel
                encoderDrive(TURN_SPEED, -9, 9, 5);
                //drive back into carousel
                encoderDrive(DRIVE_SPEED, -1.75, -1.75, 5);
                //Spin carousel
                robot.spinnerMotor.setPower(.27);
                sleep(3000);
                robot.spinnerMotor.setPower(0);
                //Strafe right to location 1
                encoderStrafe((DRIVE_SPEED - .25), 11, -11, 5.0);
                //drive forward to duck
                encoderDrive((DRIVE_SPEED - .25), 1.7, 1.8, 5);
                //Scan location 1
                telemetry.addData("Red:", robot.colorSensor.red());
                telemetry.addData("Blue:", robot.colorSensor.blue());
                telemetry.addData("Green:", robot.colorSensor.green());
                sleep(750);
                if (robot.colorSensor.red() > 25 && robot.colorSensor.blue() > 18 && robot.colorSensor.green() > 25) {// If there is yellow present at location 1
                    duckDistance = 4.625; //Then the duck is at location 1
                    sHubDistance = 17;
                    armHeight = 1200;
                } else {
                    //Drive forward to location 2
                    telemetry.addData("Red:", robot.colorSensor.red());
                    telemetry.addData("Blue:", robot.colorSensor.blue());
                    telemetry.addData("Green:", robot.colorSensor.green());
                    encoderDrive((DRIVE_SPEED - .25), 4, 4, 5.0);
                    sleep(750);
                    if (robot.colorSensor.red() > 15 && robot.colorSensor.blue() > 18 && robot.colorSensor.green() > 15) {// If there is yellow present at location 2
                        duckDistance = 6; //Then the duck is at location 2
                        sHubDistance = 13;
                        armHeight = 1100;
                    }

                }

                //back away from the duck
                encoderDrive(DRIVE_SPEED, -0.75, -0.75, 5);
                //Strafe left away from duck
                encoderStrafe(DRIVE_SPEED, 8, -8, 5.0);
                //turn a little bit to be parallel
                //encoderDrive(TURN_SPEED, -0.5, 0.5, 5);
                //drive paralell to the shub
                encoderArmDrive(DRIVE_SPEED-.3, sHubDistance, sHubDistance, 5);
                //Open armServo
                robot.armServo.setPosition(0);
                sleep(1000);
                //Change arm direction
                robot.armMotor.setDirection(DcMotor.Direction.FORWARD);
                //Drive back to parking while putting down arm
                encoderArmDrive((DRIVE_SPEED -.3),-11,-11,5);
                //Strafe right
                encoderStrafe(DRIVE_SPEED,-8,8,5);


                telemetry.addData("Path", "Complete");
                telemetry.update();


            }

            /*
             *  Method to perform a relative move, based on encoder counts.
             *  Encoders are not reset as the move is based on the current position.
             *  Move will stop if any of three conditions occur:
             *  1) Move gets to the desired position
             *  2) Move runs out of time
             *  3) Driver stops the opmode running.
             */

            public void encoderDrive(double speed,
                                     double leftInches1, double rightInches1,
                                     double timeoutS) {

                robot.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                int newLeftTarget1;
                int newRightTarget1;

                // Ensure that the opmode is still active
                if (opModeIsActive()) {

                    // Determine new target position, and pass to motor controller
                    newLeftTarget1 = robot.frontLeftMotor.getCurrentPosition() + (int) (leftInches1 * COUNTS_PER_INCH);
                    newRightTarget1 = robot.frontRightMotor.getCurrentPosition() + (int) (rightInches1 * COUNTS_PER_INCH);

                    robot.frontLeftMotor.setTargetPosition(newLeftTarget1);
                    robot.frontRightMotor.setTargetPosition(newRightTarget1);
                    robot.backLeftMotor.setTargetPosition(newLeftTarget1);
                    robot.backRightMotor.setTargetPosition(newRightTarget1);

                    // Turn On RUN_TO_POSITION
                    robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    // reset the timeout time and start motion.
                    runtime.reset();
                    robot.frontLeftMotor.setPower(Math.abs(speed));
                    robot.frontRightMotor.setPower(Math.abs(speed));
                    robot.backLeftMotor.setPower(Math.abs(speed));
                    robot.backRightMotor.setPower(Math.abs(speed));

                    // keep looping while we are still active, and there is time left, and both motors are running.
                    // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
                    // its target position, the motion will stop.  This is "safer" in the event that the robot will
                    // always end the motion as soon as possible.
                    // However, if you require that BOTH motors have finished their moves before the robot continues
                    // onto the next step, use (isBusy() || isBusy()) in the loop test.
                    while (opModeIsActive() &&
                            (runtime.seconds() < timeoutS) &&
                            (robot.frontLeftMotor.isBusy() && robot.frontRightMotor.isBusy())) {

                        // Display it for the driver.
                        telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget1, newRightTarget1);
                        telemetry.addData("Path2", "Running at %7d :%7d",
                                robot.frontLeftMotor.getCurrentPosition(),
                                robot.frontRightMotor.getCurrentPosition(),
                                robot.backLeftMotor.getCurrentPosition(),
                                robot.backRightMotor.getCurrentPosition());
                        telemetry.update();
                    }

                    // Stop all motion;
                    robot.frontLeftMotor.setPower(0.5);
                    robot.frontRightMotor.setPower(0.5);
                    robot.backLeftMotor.setPower(0.5);
                    robot.backRightMotor.setPower(0.5);

                    sleep(100);

                    robot.frontLeftMotor.setPower(0);
                    robot.frontRightMotor.setPower(0);
                    robot.backLeftMotor.setPower(0);
                    robot.backRightMotor.setPower(0);


                    // Turn off RUN_TO_POSITION
                    robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                    sleep(250);   // optional pause after each move

                    //testing computer at school
                }
            }

            public void encoderStrafe(double speed,
                                      double leftInches, double rightInches,
                                      double timeoutS) {

                robot.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                int newLeftTarget;
                int newRightTarget;

                // Ensure that the opmode is still active
                if (opModeIsActive()) {

                    // Determine new target position, and pass to motor controller
                    newLeftTarget = robot.frontLeftMotor.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
                    newRightTarget = robot.frontRightMotor.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);

                    robot.frontLeftMotor.setTargetPosition(newLeftTarget);
                    robot.frontRightMotor.setTargetPosition(newRightTarget);
                    robot.backLeftMotor.setTargetPosition(newRightTarget);
                    robot.backRightMotor.setTargetPosition(newLeftTarget);

                    // Turn On RUN_TO_POSITION
                    robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    // reset the timeout time and start motion.
                    runtime.reset();
                    robot.frontLeftMotor.setPower(Math.abs(speed));
                    robot.frontRightMotor.setPower(Math.abs(speed));
                    robot.backLeftMotor.setPower(Math.abs(speed));
                    robot.backRightMotor.setPower(Math.abs(speed));

                    // keep looping while we are still active, and there is time left, and both motors are running.
                    // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
                    // its target position, the motion will stop.  This is "safer" in the event that the robot will
                    // always end the motion as soon as possible.
                    // However, if you require that BOTH motors have finished their moves before the robot continues
                    // onto the next step, use (isBusy() || isBusy()) in the loop test.
                    while (opModeIsActive() &&
                            (runtime.seconds() < timeoutS) &&
                            (robot.frontLeftMotor.isBusy() && robot.frontRightMotor.isBusy())) {

                        // Display it for the driver.
                        telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                        telemetry.addData("Path2", "Running at %7d :%7d",
                                robot.frontLeftMotor.getCurrentPosition(),
                                robot.frontRightMotor.getCurrentPosition(),
                                robot.backLeftMotor.getCurrentPosition(),
                                robot.backRightMotor.getCurrentPosition());
                        telemetry.update();
                    }

                    // Stop all motion;
                    robot.frontLeftMotor.setPower(0);
                    robot.frontRightMotor.setPower(0);
                    robot.backLeftMotor.setPower(0);
                    robot.backRightMotor.setPower(0);

                    // Turn off RUN_TO_POSITION
                    robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                    sleep(250);   // optional pause after each move

                    robot.frontLeftMotor.setTargetPosition(newLeftTarget);
                    robot.frontRightMotor.setTargetPosition(newRightTarget);
                    robot.backLeftMotor.setTargetPosition(newLeftTarget);
                    robot.backRightMotor.setTargetPosition(newRightTarget);
                    //testing computer at school
                }

            }

            public void encoderArmDrive(double speed,
                                        double leftInches, double rightInches,
                                        double timeoutS) {

                robot.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                int newLeftTarget;
                int newRightTarget;

                // Ensure that the opmode is still active
                if (opModeIsActive()) {

                    // Determine new target position, and pass to motor controller
                    newLeftTarget = robot.frontLeftMotor.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
                    newRightTarget = robot.frontRightMotor.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);

                    robot.frontLeftMotor.setTargetPosition(newLeftTarget);
                    robot.frontRightMotor.setTargetPosition(newRightTarget);
                    robot.backLeftMotor.setTargetPosition(newLeftTarget);
                    robot.backRightMotor.setTargetPosition(newRightTarget);

                    // Turn On RUN_TO_POSITION
                    robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    // reset the timeout time and start motion.
                    runtime.reset();
                    robot.frontLeftMotor.setPower(Math.abs(speed));
                    robot.frontRightMotor.setPower(Math.abs(speed));
                    robot.backLeftMotor.setPower(Math.abs(speed));
                    robot.backRightMotor.setPower(Math.abs(speed));

                    // keep looping while we are still active, and there is time left, and both motors are running.
                    // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
                    // its target position, the motion will stop.  This is "safer" in the event that the robot will
                    // always end the motion as soon as possible.
                    // However, if you require that BOTH motors have finished their moves before the robot continues
                    // onto the next step, use (isBusy() || isBusy()) in the loop test.
                    while (opModeIsActive() &&
                            (runtime.seconds() < timeoutS) &&
                            (robot.frontLeftMotor.isBusy() && robot.frontRightMotor.isBusy())) {
                        //Lift
                        robot.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        robot.armMotor.setTargetPosition(armHeight);
                        robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        robot.armMotor.setPower(0.3);
                        while (opModeIsActive() && robot.armMotor.getCurrentPosition() < robot.armMotor.getTargetPosition()) {
                            telemetry.addData("encoder-armMotor", robot.armMotor.getCurrentPosition());
                            telemetry.update();
                            idle();
                            // Display it for the driver.
                            telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                            telemetry.addData("Path2", "Running at %7d :%7d",
                                    robot.frontLeftMotor.getCurrentPosition(),
                                    robot.frontRightMotor.getCurrentPosition(),
                                    robot.backLeftMotor.getCurrentPosition(),
                                    robot.backRightMotor.getCurrentPosition());
                            telemetry.update();
                        }
                        robot.armMotor.setDirection(DcMotor.Direction.REVERSE);
                        // Stop all motion;
                        robot.frontLeftMotor.setPower(0);
                        robot.frontRightMotor.setPower(0);
                        robot.backLeftMotor.setPower(0);
                        robot.backRightMotor.setPower(0);

                        // Turn off RUN_TO_POSITION
                        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                        sleep(250);   // optional pause after each move

                        robot.frontLeftMotor.setTargetPosition(newLeftTarget);
                        robot.frontRightMotor.setTargetPosition(newRightTarget);
                        robot.backLeftMotor.setTargetPosition(newLeftTarget);
                        robot.backRightMotor.setTargetPosition(newRightTarget);
                        //testing computer at school
                    }

                }
            }
        }



