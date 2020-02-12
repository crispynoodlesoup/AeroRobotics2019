package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
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

@Autonomous(name="RedColorSensorAutoDos", group="generated")
public class ColorAuto_Red2 extends LinearOpMode {
        //width = 18.0; //inches
        //cpr = 383.6; //counts per rotation
        //gearratio = 13.7;
        //diameter = 3.937;
        //cpi = (cpr * gearratio)/(Math.PI * diameter); //counts per inch, cpr * gear ratio / (2 * pi * diameter)
        //bias = 0.205;
        int conversion = 89;
        
        int colorOffset  = 0;
        int colorOffset2 = 0;
        int colorOffset3 = 0;
        
        int r;
        
        Boolean exit = false;
        double redSide = 1.0;
        ElapsedTime runtime = new ElapsedTime();
        GyroStuff gyro = new GyroStuff();
        HardwareMecanumbot robot = new HardwareMecanumbot();
        @Override
        public void runOpMode(){
                robot.initDrive(this);
                gyro.initDrive(robot);
                
                //robot.liftPos(17000);
                //sleep(500);
                robot.servoGrab1.setPosition(1);
                robot.servoGrab2.setPosition(0.6);
                robot.servoArm.setPosition(0.3);
                robot.eatTray(false);
                //
                waitForStart();
                //take a skystone
                strafeToPosition(-32, 1, 0);
                scanColor();
                strafeToPosition(-1.5, 1, 0);
                robot.servoArm.setPosition(0.95);
                sleep(600);
                //murder the tray!
                strafeToPosition(8, 1, 0);
                turnToTarget(90);
                strafeToPosition(80*redSide + colorOffset*redSide, 1, 90);
                robot.servoArm.setPosition(0.3);
                sleep(600);
                moveToPosition(6.0*redSide + colorOffset2*redSide, 0.5, 90);
                robot.eatTray(true);
                sleep(800);
                moveToPosition(-18, 1.0, 90);
                turnToTarget(-95);
                robot.liftPos(-5000);
                robot.eatTray(false);
                sleep(800);
                moveToPosition(-2.5, 1.0, -90);
                robot.eatTray(true);
                moveToPosition(2.0, 1.0, -90);
                sleep(3000);
                }
        public void moveToPosition(double inches, double speed, double target){
                int move  = (int)(Math.round(inches * conversion)) + 30;
                int start = robot.leftFront.getCurrentPosition();
                int end   = robot.leftFront.getCurrentPosition() + move;
                int decelInches = (int)Math.abs(Math.round(move * 9/14));
                decelInches = (int)Range.clip(decelInches, 0, 900);
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
                if(Math.abs(robot.leftFront.getCurrentPosition() - start) < 500) 
                        speed = (robot.leftFront.getCurrentPosition()*r - start*r)/500.0;
                speed = Range.clip(speed, 0.2, 1.0);
                driveStraight(target,speed,r);
                //
                //while (((robot.leftF + robot.rightF + robot.leftR + robot.rightR)/4.0 > 0.1 || end*r - robot.leftFront.getCurrentPosition()*r > 90) && !isStopRequested()){
                //while ((robot.leftF + robot.rightF + robot.leftR + robot.rightR)/4.0 > 0.1 && !isStopRequested()){
                while (end*r - robot.leftFront.getCurrentPosition()*r > 30 && !isStopRequested()){
                        speed = speedTemp;
                        if(Math.abs(robot.leftFront.getCurrentPosition()*r - start*r) < 500) 
                                speed = (robot.leftFront.getCurrentPosition()*r - start*r)/500.0 + 0.101;
                        if(Math.abs(end*r - robot.leftFront.getCurrentPosition()*r) < decelInches) 
                                speed = (end*r - robot.leftFront.getCurrentPosition()*r)/900.0;
                        speed = Range.clip(speed, 0.3, 1.0);
                        driveStraight(target,speed,r);
                        telemetry.addData("pidcalc", "%.3f", gyro.calcPID(0));
                        telemetry.addData("leftF", "%.3f", robot.leftF);
                        telemetry.addData("rightF", "%.3f", robot.rightF);
                        telemetry.addData("leftR", "%.3f", robot.leftR);
                        telemetry.addData("rightR", "%.3f", robot.rightR);
                        telemetry.addData("end", "%d", (end*r - robot.leftFront.getCurrentPosition()*r));
                        telemetry.update();
                }
                turnToTarget(target);
        }
        public void strafeToPosition(double inches, double speed, double target){
                int move  = (int)(Math.round(inches * conversion)) + 30;
                int start = robot.leftFront.getCurrentPosition();
                int end   = robot.leftFront.getCurrentPosition() + move;
                int decelInches = (int)Math.abs(Math.round(move * 9/14));
                decelInches = (int)Range.clip(decelInches, 0, 900);
                double speedTemp = speed;
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
                if(Math.abs(robot.leftFront.getCurrentPosition() - start) < 500) 
                        speed = (robot.leftFront.getCurrentPosition()*r - start*r)/500.0;
                speed = Range.clip(speed, 0.2, 1.0);
                driveStraight(target,speed,r);
                //
                //while (((robot.leftF + robot.rightF + robot.leftR + robot.rightR)/4.0 > 0.1 || end*r - robot.leftFront.getCurrentPosition()*r > 90) && !isStopRequested()){
                //while ((robot.leftF + robot.rightF + robot.leftR + robot.rightR)/4.0 > 0.1 && !isStopRequested()){
                while (end*r - robot.leftFront.getCurrentPosition()*r > 30 && !isStopRequested()){
                        speed = speedTemp;
                        if(Math.abs(robot.leftFront.getCurrentPosition()*r - start*r) < 500) 
                                speed = (robot.leftFront.getCurrentPosition()*r - start*r)/500.0 + 0.101;
                        if(Math.abs(end*r - robot.leftFront.getCurrentPosition()*r) < decelInches) 
                                speed = (end*r - robot.leftFront.getCurrentPosition()*r)/900.0;
                        speed = Range.clip(speed, 0.3, 1.0);
                        driveStraight(target,speed,r);
                        telemetry.addData("pidcalc", "%.3f", gyro.calcPID(0));
                        telemetry.addData("leftF", "%.3f", robot.leftF);
                        telemetry.addData("rightF", "%.3f", robot.rightF);
                        telemetry.addData("leftR", "%.3f", robot.leftR);
                        telemetry.addData("rightR", "%.3f", robot.rightR);
                        telemetry.addData("end", "%d", (end*r - robot.leftFront.getCurrentPosition()*r));
                        telemetry.update();
                }
                turnToTarget(target);
        }
        public void turnToTarget(double target) {
                robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                
                for(int i = 0; i < 2; i++) {
                        robot.moveLateral(0, -gyro.calcPID(target), 0, 0);
                        while (gyro.calcPID(target) != 0 && !isStopRequested()){
                                robot.moveLateral(0, -gyro.calcPID(target), 0, 0);
                                telemetry.addData("pidcalc", "%.3f", gyro.calcPID(target));
                                telemetry.update();
                        }
                        sleep(30);
                }
        }
        public void driveArc(double target, double arcLength) {
                robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                
                arcLength += 1;
                double turnDir = Range.clip(-gyro.calcPID(target)*1000, -1, 1);
                double turnSpeed = 1 / arcLength;
                
                //correction
                turnSpeed *= 1.1 + 0.035*arcLength;
                
                robot.moveLateral(1, turnSpeed*2*turnDir, 0, 0);
                while(gyro.calcPID(target) != 0 && !isStopRequested()){
                        robot.moveLateral(1, turnSpeed*2*turnDir, 0, 0);
                }
                turnToTarget(target);
        }
        public void liftPos(int pos) {
                robot.lift.setTargetPosition(pos);
                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.lift.setPower(1);
                while(robot.lift.isBusy()){}
        }
        public void driveStraight(double target, double speed, int reverse) {
                robot.moveLateral(speed, -gyro.calcPID(target)*reverse, 0, 0);
        }
        public void straifStraight(double target, double speed, int reverse) {
                robot.moveLateral(0, -gyro.calcPID(target)*reverse, speed, 0);
        }
        public void scanColor() {
                int cnt = 0;
                robot.color.enableLed(false);
                while(robot.color.red() > 35 && cnt < 2){
                        colorOffset += 8;
                        cnt += 1;
                        moveToPosition(-8*redSide, 0.6, 0);
                        telemetry.addData("combined", "%d", robot.color.argb());
                        telemetry.addData("red", "%d", robot.color.red());
                        telemetry.addData("green", "%d", robot.color.green());
                        telemetry.addData("blue", "%d", robot.color.blue());
                        telemetry.update();
                }
                if(cnt == 0)
                        colorOffset2 = -29;
        }
}
