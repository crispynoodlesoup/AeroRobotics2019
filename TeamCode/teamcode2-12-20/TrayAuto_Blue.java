package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import java.lang.annotation.Target;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

//Essentially a template auto for testing and organization

@Autonomous(name="TrayAutoBlue", group="generated")
public class TrayAuto_Blue extends LinearOpMode {
        //conversions for inches, straifConv since straifing is a tiny bit wonky
        int conversion = 88;
        int straifConv = 89;
        int speed = 1;
        
        int colorOffset = 0;
        int r; 
        
        double redSide = -1.0;
        ElapsedTime runtime = new ElapsedTime();
        GyroStuff gyro = new GyroStuff();
        HardwareMecanumbot robot = new HardwareMecanumbot();
        @Override
        public void runOpMode(){
                robot.initDrive(this);
                gyro.initDrive(robot);
                
                //initialize servos
                robot.servoGrab1.setPosition(1);
                robot.servoGrab2.setPosition(0.6);
                robot.servoArm.setPosition(0.32);
                robot.servoTray1.setPosition(1);
                robot.servoTray2.setPosition(0);
                
                //wait and execute
                waitForStart();
                robot.servoArm.setPosition(0.7);
                strafeToPosition(10.0*redSide, 0.5);
                moveToPosition(32.0, 0.6);
                sleep(800);
                robot.eatTray(true);
                sleep(800);
                driveArc(-90*redSide, -0.85, 0.4*redSide);
                robot.eatTray(false);
                sleep(800);
                moveToPosition(-1.5, 1.0);
                robot.eatTray(true);
                sleep(800);
                moveToPosition(16.0, 1.0);
                moveToPosition(-4.0, 1.0);
                turnToTarget(0);
                sleep(1000);
        }
        public void moveToPosition(double inches, double speed){
                int move  = (int)(Math.round(inches * conversion)) + 30;
                int start = robot.leftFront.getCurrentPosition();
                int end   = robot.leftFront.getCurrentPosition() + move;
                int decelInches = (int)Math.abs(Math.round(move * 18/26));
                decelInches = (int)Range.clip(decelInches, 0, 1800);
                double speedTemp = speed;
                //
                robot.leftFront.setTargetPosition(robot.leftFront.getCurrentPosition() + move);
                robot.rightFront.setTargetPosition(robot.rightFront.getCurrentPosition() + move);
                robot.leftRear.setTargetPosition(robot.leftRear.getCurrentPosition() + move);
                robot.rightRear.setTargetPosition(robot.rightRear.getCurrentPosition() + move);
                //
                robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //
                if(inches < 0)
                        r = -1;
                else
                        r = 1;
                //
                if(Math.abs(robot.leftFront.getCurrentPosition() - start) < 800) 
                        speed = (robot.leftFront.getCurrentPosition()*r - start*r)/800.0 + 0.101;
                speed = Range.clip(speed, 0.1, 1.0);
                driveStraight(0,speed,r);
                //
                //while (((robot.leftF + robot.rightF + robot.leftR + robot.rightR)/4.0 > 0.1 || end*r - robot.leftFront.getCurrentPosition()*r > 90) && !isStopRequested()){
                //while ((robot.leftF + robot.rightF + robot.leftR + robot.rightR)/4.0 > 0.1 && !isStopRequested()){
                while (end*r - robot.leftFront.getCurrentPosition()*r > 30 && !isStopRequested()){
                        speed = speedTemp;
                        if(Math.abs(robot.leftFront.getCurrentPosition()*r - start*r) < 800) 
                                speed = (robot.leftFront.getCurrentPosition()*r - start*r)/800.0 + 0.101;
                        if(Math.abs(end*r - robot.leftFront.getCurrentPosition()*r) < decelInches) 
                                speed = (end*r - robot.leftFront.getCurrentPosition()*r)/1800.0;
                        speed = Range.clip(speed, 0.1, 1.0);
                        driveStraight(0,speed,r);
                        telemetry.addData("pidcalc", "%.3f", gyro.calcPID(0));
                        telemetry.addData("leftF", "%.3f", robot.leftF);
                        telemetry.addData("rightF", "%.3f", robot.rightF);
                        telemetry.addData("leftR", "%.3f", robot.leftR);
                        telemetry.addData("rightR", "%.3f", robot.rightR);
                        telemetry.addData("end", "%d", (end*r - robot.leftFront.getCurrentPosition()*r));
                        telemetry.update();
                }
                turnToTarget(0);
        }
        public void strafeToPosition(double inches, double speed){
                //
                int move = (int)(Math.round(inches * straifConv));
                int start = robot.leftFront.getCurrentPosition();
                int end   = robot.leftFront.getCurrentPosition() + move;
                //
                robot.leftFront.setTargetPosition(robot.leftFront.getCurrentPosition() + move);
                robot.rightFront.setTargetPosition(robot.rightFront.getCurrentPosition() - move);
                robot.leftRear.setTargetPosition(robot.leftRear.getCurrentPosition() - move);
                robot.rightRear.setTargetPosition(robot.rightRear.getCurrentPosition() + move);
                //
                robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //
                if(inches < 0)
                        r = -1;
                else
                        r = 1;
                //
                if(Math.abs(robot.leftFront.getCurrentPosition() - start) < 100) 
                        speed = (robot.leftFront.getCurrentPosition()*r - start*r)/100.0;
                speed = Range.clip(speed, 0.1, 1);
                straifStraight(0, speed, r);
                //
                while (robot.leftRear.isBusy() || robot.rightFront.isBusy() || robot.leftFront.isBusy() || robot.rightRear.isBusy()){
                        if(Math.abs(robot.leftFront.getCurrentPosition() - start) < 100) 
                                speed = (robot.leftFront.getCurrentPosition()*r - start*r)/100.0;
                        if(Math.abs(end - robot.leftFront.getCurrentPosition()) < 1500) 
                                speed = (end*r - robot.leftFront.getCurrentPosition()*r)/1600.0;
                        speed = Range.clip(speed, 0.1, 1);
                        straifStraight(0, speed, r);
                        telemetry.addData("pidcalc", "%.3f", gyro.calcPID(0));
                        telemetry.update();
                }
                turnToTarget(0);
        }
        public void turnToTarget(double target) {
                robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                
                for(int i = 0; i < 5; i++) {
                        robot.moveLateral(0, -gyro.calcPID(target), 0, 0);
                        while (gyro.calcPID(target) != 0 && !isStopRequested()){
                                robot.moveLateral(0, -gyro.calcPID(target), 0, 0);
                                telemetry.addData("pidcalc", "%.3f", gyro.calcPID(target));
                                telemetry.update();
                        }
                        sleep(20);
                }
        }
        public void driveArc(double target, double speed, double turnSpeed) {
                robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                
                int originalDir = (int)Math.round(Range.clip(-gyro.calcPID(target)*10000000, -1, 1));
                int turnDir = 1;
                
                while(Math.abs(gyro.calcPID(target)) != 0 && !isStopRequested()) {
                        robot.moveLateral(speed, turnSpeed*2*turnDir, 0, 0);
                        if(originalDir != (int)Math.round(Range.clip(-gyro.calcPID(target)*10000000, -1, 1)))
                                turnDir = -1;
                        else 
                                turnDir = 1;
                }
                turnToTarget(target);
        }
        public void driveStraight(double target, double speed, int reverse) {
                robot.moveLateral(speed, -gyro.calcPID(target)*reverse, 0, 0);
        }
        public void straifStraight(double target, double speed, int reverse) {
                robot.moveLateral(0, gyro.calcPID(target)*reverse, speed, 0);
        }
}