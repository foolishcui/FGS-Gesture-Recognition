// You can make it
package gestureRecognition.fgs;

import processing.core.*;
import processing.serial.*;
import static java.lang.System.currentTimeMillis;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;

public class FGS_GestureRecognition {
    private PApplet parent;

    private Serial myPort; // Serial Object
    private boolean connected;

    private int txA;
    private int rxA;
    private int[][] rawValuesA;
    private boolean changesbyA;
    private int touchThreshold_A; // not need this value anymore, because calibration gives different threshold for each point

    private int txB;
    private int rxB;
    private int[][] rawValuesB;
    private boolean changesbyB;
    private int touchThreshold_B; // not need this value anymore, because calibration gives different threshold for each point

    // Calculate frame rate
    private int frame_count = 0;
    private long starting_time;
    private int outputCount = 0;
    private long starting_time_forOutputCount;

    // Parameters for distinguish gestures for both Sensor A ans Sensor B
    public int TAPMINDURATION;
    public int TAPMAXDURATION;
    public int DOUBLETAPMAXDELAY;
    public float SWIPEMINDISTANCE;
    public int SWIPEMAXDURATION;
    public float SLIDEMINDISTANCE;
    public int TOUCHHOLDMINDURATION;

    // SensorA 的变量设置
    public int sensorAGesture = 100;
    private int touchDurationA = 0;
    private int notTouchDuration = 0;
    private float touchDistanceA = 0.0f;
    private int initialX;
    private int initialY;
    private int currentX;
    private int currentY;
    private int prevCurrentX;
    private int prevCurrentY;
    private int touchCount = 0;
    private int notTouchCount = 0;
    private int tapCount = 0;

    // 记录slide手势与rotate手势的坐标点
    private int x0 = 0;
    private int y0 = 0;
    private int x1 = 0;
    private int y1 = 0;
    private int x2 = 0;
    private int y2 = 0;
    private int x3 = 0;
    private int y3 = 0;

    private static final int NO_GESTURE = 100;
    private static final int A_TAP = 101;
    private static final int A_DOUBLETAP = 102;
    private static final int A_TRIPLETAP = 103;
    private static final int A_SWIPEUP = 104;
    private static final int A_SWIPEDOWN = 105;
    private static final int A_SWIPELEFT = 106;
    private static final int A_SWIPERIGHT = 107;
    //    private static final int A_SLIDE = 108;
    private static final int A_ROTATE_CLOCKWISE = 109;
    private static final int A_ROTATE_COUNTERCLOCKWISE = 110;
    private static final int A_TOUCH_HOLD = 111;
    private static final int A_SLIDE_UP = 112;
    private static final int A_SLIDE_DOWN = 113;
    private static final int A_SLIDE_LEFT = 114;
    private static final int A_SLIDE_RIGHT = 115;

    // Variables for calibration and set baseline
    private boolean baseLineSet = false;
    private boolean touchBaseLineSetforA = false;
    private long calibration_waitting_time = 5000; // Calbration for 5 seconds
    private long first_calibration_done_time;
    private int[][] average_countA; // for counting
    private int[][] baseNoiseValuesA; // store baseline values for each point for Sensor A after calibration
    private int[][] maxNoiseValuesA; // store max noise values for each point for Sensor A after calibration
    private int[][] thresholdForTouchA; // If a value higher than this is detected it is considered a touch and will get adjusted
    private int[][] touchThresholdForTouchA; // store the smallest values for each point for Sensor A during touch
    private int[][] blindPointsA; // Calculate blind points
    private boolean readAdata = false;
    private int[][] arrayThresholdForTouchA;
    private int[][] arrayBlindPointsA;
    private int[][] arrayMaxNoiseValuesA;
    private int[][] arrayTouchThresholdForTouchA;
    private float thresholdfortouchA = 0.2f; // a very important value to set threshold
    public int breakpointsfortouchA = 3; // connecting break points, default is 3

    // SensorB 的变量设置
    public int sensorBGesture = 120; // SensorB的手势标识符从120开始，SensorA从100开始
    private int touchDurationB = 0;
    private int notTouchDurationB = 0;
    private float touchDistanceB = 0.0f;
    private int initialXB;
    private int initialYB;
    private int currentXB;
    private int currentYB;
    private int touchCountB = 0;
    private int notTouchCountB = 0;
    private int tapCountB = 0;

    private static final int B_NO_GESTURE = 120;
    private static final int B_TAP = 121;
    private static final int B_DOUBLETAP = 122;
    private static final int B_TRIPLETAP = 123;
    private static final int B_SLIDE_UP = 124;
    private static final int B_SLIDE_DOWN = 125;
    private static final int B_TOUCH_HOLD = 126;

    private boolean touchBaseLineSetforB = false;
    private int[][] average_countB;
    private int[][] baseNoiseValuesB;
    private int[][] maxNoiseValuesB;
    private int[][] thresholdForTouchB;
    private int[][] touchThresholdForTouchB;
    private int[][] blindPointsB;
    private boolean readBdata = false;
    private int[][] arrayThresholdForTouchB;
    private int[][] arrayBlindPointsB;
    private int[][] arrayMaxNoiseValuesB;
    private int[][] arrayTouchThresholdForTouchB;
    private float thresholdfortouchB = 0.2f; // a very important value to set threshold
    public int breakpointsfortouchB = 3; // connecting break points, default is 3

    // for more specific control and visualizatino
    private String lastSensorAGestureName = "A_NO_GESTURE";
    private String lastSensorBGestureName = "B_NO_GESTURE";
    private boolean isATouchDown = false;
    private boolean rotateClockwise = false;
    private boolean rotateCounterClockwise = false;
    private float maxdistance = 0; // 获取划动的距离过程中的最大值，用于区分rotate与slide

    // Constructor，使用这个构造函数时使用默认的手势识别默认变量
    public FGS_GestureRecognition(PApplet parent, int txA, int rxA, int txB, int rxB, int serialPort) {
        this.parent = parent;
        this.txA = txA;
        this.rxA = rxA;
        this.txB = txB;
        this.rxB = rxB;

        this.rawValuesA = new int [txA][rxA];
        this.average_countA = new int [txA][rxA];
        this.baseNoiseValuesA = new int [txA][rxA];
        this.maxNoiseValuesA = new int [txA][rxA];
        this.thresholdForTouchA = new int [txA][rxA];
        this.touchThresholdForTouchA = new int [txA][rxA];
        this.blindPointsA = new int [txA][rxA];
        this.arrayThresholdForTouchA = new int [txA][rxA];
        this.arrayBlindPointsA = new int [txA][rxA];
        this.arrayMaxNoiseValuesA = new int [txA][rxA];
        this.arrayTouchThresholdForTouchA = new int [txA][rxA];

        this.rawValuesB = new int [txB][rxB];
        this.average_countB = new int [txB][rxB];
        this.baseNoiseValuesB = new int [txB][rxB];
        this.maxNoiseValuesB = new int [txB][rxB];
        this.thresholdForTouchB = new int [txB][rxB];
        this.touchThresholdForTouchB = new int [txB][rxB];
        this.blindPointsB = new int [txB][rxB];
        this.arrayThresholdForTouchB = new int [txB][rxB];
        this.arrayBlindPointsB = new int [txB][rxB];
        this.arrayMaxNoiseValuesB = new int [txB][rxB];
        this.arrayTouchThresholdForTouchB = new int [txB][rxB];

        this.TAPMINDURATION = 20;
        this.TAPMAXDURATION = 100;
        this.DOUBLETAPMAXDELAY = 100;
        this.SWIPEMINDISTANCE = 3.0f;
        this.SWIPEMAXDURATION = 300;
        this.SLIDEMINDISTANCE = 2.0f;

        // creating the font to draw on screen later
        PFont f;
        f = parent.createFont("Arial",32,true); // Arial, 16 point, anti-aliasing on
        parent.textFont(f,15);

        connect(serialPort);
//        parent.registerMethod("draw", this);
    }

    // Constructor，尽量使用这个构造函数，可以在Processing中调节控制手势识别的变量
    public FGS_GestureRecognition(PApplet parent, int txA, int rxA, int txB, int rxB, int serialPort, int TAPMINDURATION, int TAPMAXDURATION, int DOUBLETAPMAXDELAY, float SWIPEMINDISTANCE, int SWIPEMAXDURATION, float SLIDEMINDISTANCE, int TOUCHHOLDMINDURATION, float thresholdfortouchA, float thresholdfortouchB, int breakpointsfortouchA, int breakpointsfortouchB) {
        this.parent = parent;
        this.txA = txA;
        this.rxA = rxA;
        this.txB = txB;
        this.rxB = rxB;

        this.thresholdfortouchA = thresholdfortouchA;
        this.thresholdfortouchB = thresholdfortouchB;
        this.breakpointsfortouchA = breakpointsfortouchA;
        this.breakpointsfortouchB = breakpointsfortouchB;

        this.rawValuesA = new int [txA][rxA];
        this.average_countA = new int [txA][rxA];
        this.baseNoiseValuesA = new int [txA][rxA];
        this.maxNoiseValuesA = new int [txA][rxA];
        this.thresholdForTouchA = new int [txA][rxA];
        this.touchThresholdForTouchA = new int [txA][rxA];
        this.blindPointsA = new int [txA][rxA];
        this.arrayThresholdForTouchA = new int [txA][rxA];
        this.arrayBlindPointsA = new int [txA][rxA];
        this.arrayMaxNoiseValuesA = new int [txA][rxA];
        this.arrayTouchThresholdForTouchA = new int [txA][rxA];

        this.rawValuesB = new int [txB][rxB];
        this.average_countB = new int [txB][rxB];
        this.baseNoiseValuesB = new int [txB][rxB];
        this.maxNoiseValuesB = new int [txB][rxB];
        this.thresholdForTouchB = new int [txB][rxB];
        this.touchThresholdForTouchB = new int [txB][rxB];
        this.blindPointsB = new int [txB][rxB];
        this.arrayThresholdForTouchB = new int [txB][rxB];
        this.arrayBlindPointsB = new int [txB][rxB];
        this.arrayMaxNoiseValuesB = new int [txB][rxB];
        this.arrayTouchThresholdForTouchB = new int [txB][rxB];

        this.TAPMINDURATION = TAPMINDURATION;
        this.TAPMAXDURATION = TAPMAXDURATION;
        this.DOUBLETAPMAXDELAY = DOUBLETAPMAXDELAY;
        this.SWIPEMINDISTANCE = SWIPEMINDISTANCE;
        this.SWIPEMAXDURATION = SWIPEMAXDURATION;
        this.SLIDEMINDISTANCE = SLIDEMINDISTANCE;
        this.TOUCHHOLDMINDURATION = TOUCHHOLDMINDURATION;

        PFont f; // creating the font to draw on screen later
        f = parent.createFont("Arial",32,true); // Arial, 16 point, anti-aliasing on
        parent.textFont(f,24);
        connect(serialPort);
//        parent.registerMethod("draw", this);
    }

    // 校准SensorA和SensorB，在Processing中使用，之前需要执行readSerial()
    public void calibrateSensor_AB() {
        if (changesbyA || changesbyB) { // 这里注意是"或"就开始校准，不可能同时为true
            if (!baseLineSet) {
                calibrateNoiseForSensor(); // 计算SensorA与SensorB每个点噪音的平均值与最大值，这个步骤是同时进行的
            } else if (baseLineSet && !touchBaseLineSetforA) {
                // touch down 做为第二次校准过程，这个过程必须先后进行, 因为需要手分别放在传感器上
                calibrateTouchDownForSensorA(); // 计算SensorA每个点touch down时发生电容耦合的电压值
            } else if (baseLineSet && touchBaseLineSetforA && !touchBaseLineSetforB) {
                calibrateTouchDownForSensorB(); // 计算SensorB每个点touch down时发生电容耦合的电压值
            } else if (baseLineSet && touchBaseLineSetforA && touchBaseLineSetforB) {
//                showMeSomething(); // 这个函数目前设置为读取文件数组
            }
        }
    }

    // 计算SensorA和SensorB的手势（在校准完成后），在Processing中使用，之前需要执行readSerial()
    public void getGesture_AB() {
//        if (baseLineSet && touchBaseLineSetforA && touchBaseLineSetforB) { //如何读取文件时不需要此行，如果不读取文件取消掉注释
            if (changesbyA) {
                calcSensorAGesture();
            }
            if (changesbyB) {
                calcSensorBGesture();
            }
//        }
    }

    // 计算SensorA的手势
    public void calcSensorAGesture() {

        if ( isSensorA_TouchDown() ) { // 如果此时Touch Down，需要完成一些初始化工作
            touchCount++;
            if (touchCount == 1) { // 完成初始化与重置的工作
                initialX = currentX;
                initialY = currentY;
                notTouchDuration = notTouchCount; // 记录notTouchDuration 是为了判断多次点击的手势
                notTouchCount = 0;
                System.out.println();
                System.out.println();
            }

            System.out.println("SensorA坐标为: " + "(" + currentX + "," + currentY + ")," + "此时的电容值是: " + rawValuesA[currentX][currentY]); // 输出坐标和此时的电容值:
            touchDurationA = touchCount;
            touchDistanceA = (float) Math.sqrt(Math.pow(currentX - initialX, 2) + Math.pow(currentY - initialY, 2));

            // A_SLIDE and A_ROTATE
            if (touchDurationA > SWIPEMAXDURATION && touchDistanceA >= SLIDEMINDISTANCE) {
                if (rotateClockwise) {
                    setSensorAGesture(A_ROTATE_CLOCKWISE, "A_ROTATE_CLOCKWISE");
                    System.out.println("A_ROTATE_CLOCKWISE");
                } else if (rotateCounterClockwise) {
                    setSensorAGesture(A_ROTATE_COUNTERCLOCKWISE, "A_ROTATE_COUNTERCLOCKWISE");
                    System.out.println("A_ROTATE_COUNTERCLOCKWISE");
                }
                // 首先需将手势分成前后两段，基于两段的arctan的大小变化判断是 Slide 还是 Rotate
//                if ((touchDurationA - SWIPEMAXDURATION) % 3 == 0 && !rotateClockwise && !rotateCounterClockwise) {
//                    x0 = x1;
//                    y0 = y1;
//                    x1 = x2;
//                    y1 = y2;
//                    x2 = x3;
//                    y2 = y3;
//                    x3 = currentX;
//                    y3 = currentY;
//                    System.out.println("x0值为: " + x0 + " y0值为: " + y0 + " x1值为: " + x1 + " y1值为: " + y1 + " x2值为: " + x2 + " y2值为: " + y2 + " x3值为: " + x3 + " y3值为: " + y3);
//                }
//                float distance3 = (float) Math.sqrt(Math.pow(x3 - initialX, 2) + Math.pow(y3 - initialY, 2));
//                float distance1 = (float) Math.sqrt(Math.pow(x1 - initialX, 2) + Math.pow(y1 - initialY, 2));
//                double degreeDifference = Math.atan2(y3 - y2, x3 - x2) * 180 / Math.PI - Math.atan2(y1 - y0, x1 - x0) * 180 / Math.PI; // 计算arctan差值
//                System.out.println("线段角度的差值是: " + degreeDifference);
                if (touchDistanceA > maxdistance) { // 找到划动过程中的touchDistanceA最大值
                    maxdistance = touchDistanceA;
                }
                // 后面可以使用模板匹配的方法来识别圆
                if (touchDistanceA - maxdistance < -3.0 && !rotateClockwise && !rotateCounterClockwise) {
                    System.out.println("touchDistanceA的值为: " + touchDistanceA);
                    System.out.println("maxdistance的值为: " + maxdistance);
                    if (currentX-initialX > 0) {
                        rotateClockwise = true;
                        rotateCounterClockwise = false;
                        setSensorAGesture(A_ROTATE_CLOCKWISE, "A_ROTATE_CLOCKWISE");
                        System.out.println("A_ROTATE_CLOCKWISE");
                    } else if (currentX-initialX < 0) {
                        rotateCounterClockwise = true;
                        rotateClockwise = false;
                        setSensorAGesture(A_ROTATE_COUNTERCLOCKWISE, "A_ROTATE_COUNTERCLOCKWISE");
                        System.out.println("A_ROTATE_COUNTERCLOCKWISE");
                    }
                } else {
                    if (sensorAGesture != A_TOUCH_HOLD && sensorAGesture != A_ROTATE_CLOCKWISE && sensorAGesture != A_ROTATE_COUNTERCLOCKWISE) {
                        if (Math.abs(currentY - initialY) >= Math.abs(currentX - initialX) && currentY - initialY < 0) {
                            if (sensorAGesture != A_SLIDE_RIGHT && sensorAGesture != A_SLIDE_LEFT && sensorAGesture != A_SLIDE_DOWN) {
                                setSensorAGesture(A_SLIDE_UP, "A_SLIDE_UP");
                                System.out.println("A_SLIDE_UP");
                            }
                        } else if (Math.abs(currentY - initialY) >= Math.abs(currentX - initialX) && currentY - initialY > 0) {
                            if (sensorAGesture != A_SLIDE_LEFT && sensorAGesture != A_SLIDE_UP && sensorAGesture != A_SLIDE_RIGHT) {
                                setSensorAGesture(A_SLIDE_DOWN, "A_SLIDE_DOWN");
                                System.out.println("A_SLIDE_DOWN");
                            }
                        } else if (Math.abs(currentY - initialY) < Math.abs(currentX - initialX) && currentX - initialX > 0) {
                            if (sensorAGesture != A_SLIDE_DOWN && sensorAGesture != A_SLIDE_UP && sensorAGesture != A_SLIDE_RIGHT) {
                                setSensorAGesture(A_SLIDE_LEFT, "A_SLIDE_LEFT");
                                System.out.println("A_SLIDE_LEFT");
                            }
                        } else if (Math.abs(currentY - initialY) < Math.abs(currentX - initialX) && currentX - initialX < 0) {
                            if (sensorAGesture != A_SLIDE_DOWN && sensorAGesture != A_SLIDE_UP && sensorAGesture != A_SLIDE_LEFT) {
                                setSensorAGesture(A_SLIDE_RIGHT, "A_SLIDE_RIGHT");
                                System.out.println("A_SLIDE_RIGHT");
                            }
                        }
                    }
                }
            }

            // A_TOUCH_HOLD
            if (touchDurationA > TOUCHHOLDMINDURATION && touchDistanceA < SLIDEMINDISTANCE) {
                if ( sensorAGesture != A_SLIDE_UP && sensorAGesture != A_SLIDE_DOWN && sensorAGesture != A_SLIDE_LEFT && sensorAGesture != A_SLIDE_RIGHT && sensorAGesture != A_ROTATE_CLOCKWISE && sensorAGesture != A_ROTATE_COUNTERCLOCKWISE ) {
                    setSensorAGesture(A_TOUCH_HOLD, "A_TOUCH_HOLD");
                    System.out.println("A_TOUCH_HOLD");
                }
            }
        }

        else { // 如果此时没有Touch Down，需要完成一些初始化工作
            notTouchCount ++; // 记录not touch down过程的帧数
            sensorAGesture = NO_GESTURE;

            if (notTouchCount == breakpointsfortouchA + 1) { // 完成初始化与重置的工作
                touchCount = 0;
                rotateClockwise = false;
                rotateCounterClockwise = false;
                System.out.println("本次触控之前未触控的时长帧率为: " + notTouchDuration);
                System.out.println("本次触控时长帧率为: " + touchDurationA);
                System.out.println("本次触控距离为: " + touchDistanceA);
            }

            // Gesture: A_TAP, A_DOUBLETAP, A_TRIPLE_TAP, or many more consequent tap, all will be recognize A_TRIPLE_TAP
            if (touchDurationA >= TAPMINDURATION && touchDurationA <= TAPMAXDURATION && touchDistanceA < SWIPEMINDISTANCE && notTouchCount > breakpointsfortouchA) {

                if (notTouchCount == breakpointsfortouchA + 1) { tapCount++; }

                if (tapCount == 1 && notTouchDuration > DOUBLETAPMAXDELAY && notTouchCount > DOUBLETAPMAXDELAY) {
                    setSensorAGesture(A_TAP, "A_TAP");
                    System.out.println("A_TAP");
                } else if (tapCount == 2 && notTouchDuration <= DOUBLETAPMAXDELAY && notTouchCount > DOUBLETAPMAXDELAY) {
                    setSensorAGesture(A_DOUBLETAP, "A_DOUBLETAP");
                    System.out.println("A_DOUBLETAP");
                } else if (tapCount >= 3 && notTouchDuration <= DOUBLETAPMAXDELAY && notTouchCount > DOUBLETAPMAXDELAY) {
                    setSensorAGesture(A_TRIPLETAP, "A_TRIPLETAP");
                    System.out.println("A_TRIPLETAP");
                    // 当 not touch count 计数较大时，将tapCount记为0，防止之前的tap次数被用到后面的tap次数判断中
                } else if (notTouchCount > 2 * DOUBLETAPMAXDELAY) { tapCount = 0; }

                // Gesture: A_SWIPE
            } else if (touchDurationA > TAPMINDURATION && touchDurationA <= SWIPEMAXDURATION && touchDistanceA >= SWIPEMINDISTANCE && notTouchCount > breakpointsfortouchA) {
                if (Math.abs(currentY - initialY) >= Math.abs(currentX - initialX) && currentY - initialY < 0) {
                    setSensorAGesture(A_SWIPEUP, "A_SWIPEUP");
                    System.out.println("A_SWIPEUP");
                } else if (Math.abs(currentY - initialY) >= Math.abs(currentX - initialX) && currentY - initialY > 0) {
                    setSensorAGesture(A_SWIPEDOWN, "A_SWIPEDOWN");
                    System.out.println("A_SWIPEDOWN");
                } else if (Math.abs(currentY - initialY) < Math.abs(currentX - initialX) && currentX - initialX > 0) {
                    setSensorAGesture(A_SWIPELEFT, "A_SWIPELEFT");
                    System.out.println("A_SWIPELEFT");
                } else if (Math.abs(currentY - initialY) < Math.abs(currentX - initialX) && currentX - initialX < 0) {
                    setSensorAGesture(A_SWIPERIGHT, "A_SWIPERIGHT");
                    System.out.println("A_SWIPERIGHT");
                }

            } else if (notTouchCount > breakpointsfortouchA) {
                setSensorAGesture(NO_GESTURE, "NO_GESTURE"); // return NO_GESTURE of Sensor A
            }
        }
    }

    // 计算SensorB的手势
    public void calcSensorBGesture() {

        // 如果此时Touch Down，需要完成一些初始化工作
        if ( isSensorB_TouchDown() ) {

            // 输出坐标和此时的电容值
            System.out.println("SensorB当前的触碰坐标是: " + "(" + currentXB + "," + currentYB + ")," + "此时的电容值是: " + rawValuesB[currentXB][currentYB]);
            touchCountB ++;

            if (touchCountB == 1) { // 完成初始化与重置的工作
                initialXB = currentXB;
                initialYB = currentYB;
                notTouchDurationB = notTouchCountB; // 记录notTouchDuration 是为了判断多次点击的手势
                notTouchCountB = 0;
            }

            touchDurationB = touchCountB;
            touchDistanceB = Math.abs(currentXB - initialXB);

            // B_SLIDE
            if (touchDurationB > TAPMINDURATION+7 && touchDistanceB >= SLIDEMINDISTANCE) {
                System.out.println("本次触控距离为: " + touchDistanceB);
                if (currentXB - initialXB > 0) {
                    setSensorBGesture(B_SLIDE_UP, "B_SLIDE_UP");
                    System.out.println("B_SLIDE_UP");
                } else if (currentXB - initialXB < 0) {
                    setSensorBGesture(B_SLIDE_DOWN, "B_SLIDE_DOWN");
                    System.out.println("B_SLIDE_DOWN");
                }
            }

            // B_TOUCH_HOLD
            if (touchDurationB > TOUCHHOLDMINDURATION && touchDistanceB < SLIDEMINDISTANCE) {
                if ( sensorBGesture != B_SLIDE_UP && sensorBGesture != B_SLIDE_DOWN ) {
                    setSensorBGesture(B_TOUCH_HOLD, "B_TOUCH_HOLD");
                    System.out.println("B_TOUCH_HOLD");
                }
            }
        }

        // 如果此时没有Touch Down，需要完成一些初始化工作
        if ( !isSensorB_TouchDown() ) {
            notTouchCountB ++; // 记录not touch down过程的帧数
            sensorBGesture = B_NO_GESTURE;

            if (notTouchCountB == 2) { // 完成初始化与重置的工作
                touchCountB = 0;
                System.out.println("本次触控之前未触控的时长帧率为: " + notTouchDurationB);
                System.out.println("本次触控时长帧率为: " + touchDurationB);
                System.out.println("本次触控距离为: " + touchDistanceB);
            }

            // Gesture: B_TAP, B_DOUBLETAP, B_TRIPLE_TAP, or many more consequent tap, all will be recognize B_TRIPLE_TAP
            if (touchDurationB >= TAPMINDURATION && touchDurationB <= TAPMAXDURATION && touchDistanceB <= SWIPEMINDISTANCE && notTouchCountB > 1) {
                if (notTouchCountB == 2) {
                    tapCountB++;
                }
                if (tapCountB == 1 && notTouchDurationB > DOUBLETAPMAXDELAY && notTouchCountB > DOUBLETAPMAXDELAY) {
                    setSensorBGesture(B_TAP, "B_TAP");
                    System.out.println("B_TAP");
                } else if (tapCountB == 2 && notTouchDurationB <= DOUBLETAPMAXDELAY && notTouchCountB > DOUBLETAPMAXDELAY) {
                    setSensorBGesture(B_DOUBLETAP, "B_DOUBLETAP");
                    System.out.println("B_DOUBLETAP");
                } else if (tapCountB >= 3 && notTouchDurationB <= DOUBLETAPMAXDELAY && notTouchCountB > DOUBLETAPMAXDELAY) {
                    setSensorBGesture(B_TRIPLETAP, "B_TRIPLETAP");
                    System.out.println("B_TRIPLETAP");
                    // 当 not touch count 计数较大时，将tapCount记为0，防止之前的tap次数被用到后面的tap次数判断中
                } else if (notTouchCountB > 2 * DOUBLETAPMAXDELAY) {
                    tapCountB = 0;
                }

            } else if (notTouchCountB > 1) {
                setSensorBGesture(B_NO_GESTURE, "B_NO_GESTURE"); // return B_NO_GESTURE of SensorB
            }
        }
    }

    // 判断SensorA有没有被 touch down，以此返回当前的currentX与currentY
    public boolean isSensorA_TouchDown() {
        isATouchDown = false;
        if (!readAdata) { // 从文件中读取二维数组
            readAdata = true;
            arrayBlindPointsA = readArrayFile("arrayBlindPointsA");
            arrayMaxNoiseValuesA = readArrayFile("arrayMaxNoiseValuesA");
            arrayTouchThresholdForTouchA = readArrayFile("arrayTouchThresholdForTouchA");
            // 这里基于读取的文件的值，重新计算每个点的阈值
            for (int row = 0; row < txA; row ++) {
                for (int column = 0; column < rxA; column ++) {
                    arrayThresholdForTouchA[row][column] = (int) (arrayMaxNoiseValuesA[row][column] + thresholdfortouchA*(arrayTouchThresholdForTouchA[row][column] - arrayMaxNoiseValuesA[row][column]));
                }
            }
        } else {
            float capacativeRatio = 0; // 用于取最大值
            float adjustValue = 0; // 用于找到最亮的点
            prevCurrentX = currentX;
            prevCurrentY = currentY;
            for (int row = 0; row < txA; row++) {
                for (int column = 0; column < rxA; column++) {
                    // 这里排除盲点的情况，盲点位置不会出现touch的状态
                    if ( rawValuesA[row][column] > arrayThresholdForTouchA[row][column] && arrayBlindPointsA[row][column] > 0 ) {
                        isATouchDown = true;
                        // 获取当前正在碰触到的坐标, 通过map方法返回亮度最大的点找到此时触碰的坐标
                        adjustValue = parent.map(rawValuesA[row][column], arrayMaxNoiseValuesA[row][column], arrayTouchThresholdForTouchA[row][column], 0, 255);
                        if (adjustValue > capacativeRatio) {
                            capacativeRatio = adjustValue;
                            currentX = row;
                            currentY = column;
                        }
                        parent.fill(255); // just for video demo!!!
                        parent.textSize(48);
                        parent.text("SensorA: " + "A_ROTATE_CLOCKWISE", 20, 800);
                    }
                }
            }
        }
        if (isATouchDown) {
            // 找到最亮的点后寻找当前点与上次碰触的点的关系，把跳点给禁止掉
            if (touchCount == 0) {
                return true;
            } else if ( Math.abs(currentX - prevCurrentX) <= 2 && Math.abs(currentY - prevCurrentY) <= 3 ) {  //不是第一个点
                return true;
            } else {
                System.out.println("跳点的坐标为: " + "(" + currentX + "," + currentY + "),"); // 输出跳点坐标
                currentX = prevCurrentX; // 断点不可以改变currentX与currentY的值
                currentY = prevCurrentY;
                return false;
            }
        } else {
            return false;
        }
    }

    // 判断SensorB有没有被 touch down，以此返回当前的currentX与currentY
    public boolean isSensorB_TouchDown() {
        boolean isBTouchDown = false;
        if (!readBdata) {
            readBdata = true;
            arrayBlindPointsB = readArrayFile("arrayBlindPointsB");
            arrayMaxNoiseValuesB = readArrayFile("arrayMaxNoiseValuesB");
            arrayTouchThresholdForTouchB = readArrayFile("arrayTouchThresholdForTouchB");
            for (int row = 0; row < txB; row ++) {
                for (int column = 0; column < rxB; column ++) {
                    arrayThresholdForTouchB[row][column] = (int) (arrayMaxNoiseValuesB[row][column] + thresholdfortouchB*(arrayTouchThresholdForTouchB[row][column] - arrayMaxNoiseValuesB[row][column]));
                }
            }
        } else if (!isATouchDown) {
            float capacativeRatio = 0;
            float adjustValue = 0;
            for (int row = 0; row < txB; row++) {
                for (int column = 0; column < rxB; column++) {
                    // 这里排除盲点的情况，盲点位置不会出现touch的状态
                    if ( rawValuesB[row][column] > arrayThresholdForTouchB[row][column] && arrayBlindPointsB[row][column] > 0 ) {
                        isBTouchDown = true;
                        adjustValue = parent.map(rawValuesB[row][column], arrayMaxNoiseValuesB[row][column], arrayTouchThresholdForTouchB[row][column], 0, 255);
                        // 获取当前正在碰触到的坐标, 通过map方法返回亮度最大的点找到此时触碰的坐标
                        if ( adjustValue > capacativeRatio ) {
                            capacativeRatio = adjustValue;
                            currentXB = row;
                            currentYB = column;
                        }
                    }
                }
            }
        }
        if (isBTouchDown) { return true; } else { return false; }
    }

    // 这里是第一遍校准，用来记录噪音的平均值与最大值，目前设置校准时间为5秒钟，这5秒钟将SensorA与SensorB的噪音平均值与最大值全部校准完成
    public void calibrateNoiseForSensor() {
        // 在校准的5秒钟之内累加所有噪音值，并记录每个点噪音的最大值，这里用来计算SensorA每个点的平均值
        if ( currentTimeMillis() - starting_time <= calibration_waitting_time ) {
            parent.background(parent.color(0,0,0));
            parent.fill(255);
            parent.text("校准噪音中，请在5秒钟之内不要碰触传感器!", 20, 30);
            // 这个for循环计算SensorA每个点的噪音的平均值与最大值做为基线，目前选择噪音的最大值做为基线值
            // 先计算SensorA，再计算SensorB
            for (int row = 0; row < txA; row ++) {
                for (int column = 0; column < rxA; column ++) {
                    // 第1个for循环用来求和
                    baseNoiseValuesA[row][column] = baseNoiseValuesA[row][column] + rawValuesA[row][column];
                    average_countA[row][column] = average_countA[row][column] + 1;

                    // 计算校准5秒过程中的噪音的最大值，这个是 not touch 的判断标准
                    if ( rawValuesA[row][column] > maxNoiseValuesA[row][column] ) {
                        maxNoiseValuesA[row][column] = rawValuesA[row][column];
                    }
                }
            }
            // 计算SensorB的噪音平均值与最大值
            for (int row = 0; row < txB; row ++) {
                for (int column = 0; column < rxB; column ++) {
                    // 第1个for循环用来求和
                    baseNoiseValuesB[row][column] = baseNoiseValuesB[row][column] + rawValuesB[row][column];
                    average_countB[row][column] = average_countB[row][column] + 1;

                    // 计算校准5秒过程中的噪音的最大值，这个是 not touch 的判断标准
                    if ( rawValuesB[row][column] > maxNoiseValuesB[row][column] ) {
                        maxNoiseValuesB[row][column] = rawValuesB[row][column];
                    }
                }
            }
        }

        // 判断时间是否已经超过了5秒钟
        if ( currentTimeMillis() - starting_time > calibration_waitting_time ) {
            System.out.println(starting_time);
            System.out.println(currentTimeMillis() - starting_time);
            parent.noFill();
            baseLineSet = true; // 设置标识符为true
            // SensorA: 计算校准5秒过程中的基线平均值，第2个for循环用来计算每个点的平均值，这里同时在Processing中输出基线平均值
            System.out.println("基线已经设置成功, 下方数组展示了SensorA所有点的噪音平均值。");
            for (int row = 0; row < txA; row ++) {
                for (int column = 0; column < rxA; column ++) {
                    baseNoiseValuesA[row][column] = baseNoiseValuesA[row][column] / average_countA[row][column];
                    System.out.printf("%5d ", baseNoiseValuesA[row][column]);
                }
                System.out.println();
            }
            // SensorA: 这里在Processing中输出每个点噪音的最大值
            System.out.println();
            System.out.println("基线已经设置成功, 下方数组展示了SensorA所有点的最大噪音值。");
            for (int row = 0; row < txA; row ++) {
                for (int column = 0; column < rxA; column ++) {
                    System.out.printf("%5d ", maxNoiseValuesA[row][column]);
                }
                System.out.println();
            }
            System.out.println();
            // SensorB: 计算校准5秒过程中的基线平均值，第2个for循环用来计算每个点的平均值，这里同时在Processing中输出基线平均值
            System.out.println("基线已经设置成功, 下方数组展示了SensorB所有点的噪音平均值。");
            for (int row = 0; row < txB; row ++) {
                for (int column = 0; column < rxB; column ++) {
                    baseNoiseValuesB[row][column] = baseNoiseValuesB[row][column] / average_countB[row][column];
                    System.out.printf("%5d ", baseNoiseValuesB[row][column]);
                }
                System.out.println();
            }
            // SensorB: 这里在Processing中输出每个点噪音的最大值
            System.out.println();
            System.out.println("基线已经设置成功, 下方数组展示了SensorB所有点的最大噪音值。");
            for (int row = 0; row < txB; row ++) {
                for (int column = 0; column < rxB; column ++) {
                    System.out.printf("%5d ", maxNoiseValuesB[row][column]);
                }
                System.out.println();
            }

            // 写文件
            saveArrayToFile(maxNoiseValuesA, "arrayMaxNoiseValuesA");
            saveArrayToFile(maxNoiseValuesB, "arrayMaxNoiseValuesB");
            // 重新设置下一次校准的起始时间
            first_calibration_done_time = currentTimeMillis();
        }
    }

    // 这里是第二遍校准，用来记录SensorA touch down过程的最小值，目前设置校准时间为5秒钟
    public void calibrateTouchDownForSensorA() {
        if ( currentTimeMillis() - first_calibration_done_time <= 5000 ) {
            parent.background(parent.color(0,0,0));
            parent.fill(255);
            parent.text("准备进行SensorA的第二轮校准，请将手全部盖在SensorA上并稍微用力。", 20, 30);
        }
        if ( currentTimeMillis() - first_calibration_done_time > 5000 && currentTimeMillis() - first_calibration_done_time <= 10000 ) {
            parent.background(parent.color(0,0,0));
            parent.fill(255);
            parent.text("正在进行SensorA的第二轮校准, 请不要将手拿开。", 20, 30);

            // 为所有点赋初始值为1000，这里是为所有点取最小值
            if ( touchThresholdForTouchA[0][0] == 0 ) {
                // 为所有的点赋初始值为1000
                for (int row = 0; row < txA; row ++) {
                    for (int column = 0; column < rxA; column ++) {
                        touchThresholdForTouchA[row][column] = 10000;
                    }
                }
            }
            for (int row = 0; row < txA; row ++) {
                for (int column = 0; column < rxA; column ++) {
                    if ( rawValuesA[row][column] < touchThresholdForTouchA[row][column] ) {
                        touchThresholdForTouchA[row][column] = rawValuesA[row][column];
                    }
                }
            }
            // 为所有的点取最大值
//            for (int row = 0; row < txA; row ++) {
//                for (int column = 0; column < rxA; column ++) {
//                    if ( rawValuesA[row][column] > touchThresholdForTouchA[row][column] ) {
//                        touchThresholdForTouchA[row][column] = rawValuesA[row][column];
//                    }
//                }
//            }
        }

        if ( currentTimeMillis() - first_calibration_done_time > 10000 ) {
            touchBaseLineSetforA = true;
            System.out.println("触碰基线已经设置成功, 下方数组展示了SensorA所有点的触碰时的最小值。");
            parent.background(parent.color(0,0,0));
            parent.fill(255);
            parent.text("第二轮校准已完成，可以开始使用SensorA传感器识别您的手势。", 20, 30);

            // 输出校准后的每个点的值
            for (int row = 0; row < txA; row ++) {
                for (int column = 0; column < rxA; column ++) {
                    System.out.printf("%5d ", touchThresholdForTouchA[row][column]);
                }
                System.out.println();
            }
            // 如果计算出得thresholdForTouchA[][]的某个点是负的或者是0，这个点被归位盲点。
            for (int row = 0; row < txA; row ++) {
                for (int column = 0; column < rxA; column ++) {
                    blindPointsA[row][column] = touchThresholdForTouchA[row][column] - maxNoiseValuesA[row][column];
                    if (blindPointsA[row][column] > 0) {
                        blindPointsA[row][column] = 1;
                    } else {
                        blindPointsA[row][column] = 0;
                    }
                }
            }
            // 计算每个点的阈值, 这里的一个关键阈值使用了0.5，设置的较小是因为比较容易检测到 touch down
            for (int row = 0; row < txA; row ++) {
                for (int column = 0; column < rxA; column ++) {
                    thresholdForTouchA[row][column] = (int) (maxNoiseValuesA[row][column] + thresholdfortouchA*(touchThresholdForTouchA[row][column] - maxNoiseValuesA[row][column]));
                }
            }
            saveArrayToFile(touchThresholdForTouchA, "arrayTouchThresholdForTouchA");
            saveArrayToFile(blindPointsA, "arrayBlindPointsA"); // 将blindPointsA[row][column]写入文件
            saveArrayToFile(thresholdForTouchA, "arrayThresholdForTouchA"); // 将thresholdForTouchA[row][column]写入文件
        }
    }

    // 这里是第二遍校准，用来记录SensorB touch down过程的最小值，目前设置校准时间为5秒钟
    public void calibrateTouchDownForSensorB() {
        if ( currentTimeMillis() - first_calibration_done_time <= 15000 ) {
            parent.background(parent.color(0,0,0));
            parent.fill(255);
            parent.text("准备进行SensorB的第二轮校准，请将手全部盖在SensorB上并稍微用力。", 20, 30);
        }
        if ( currentTimeMillis() - first_calibration_done_time > 15000 && currentTimeMillis() - first_calibration_done_time <= 20000 ) {
            parent.background(parent.color(0,0,0));
            parent.fill(255);
            parent.text("正在进行SensorB的第二轮校准, 请不要将手拿开。", 20, 30);

            // 为所有点赋初始值为1000，这里是为所有点取最小值
            if ( touchThresholdForTouchB[0][0] == 0 ) {
                // 为所有的点赋初始值为1000
                for (int row = 0; row < txB; row ++) {
                    for (int column = 0; column < rxB; column ++) {
                        touchThresholdForTouchB[row][column] = 1000;
                    }
                }
            }
            for (int row = 0; row < txB; row ++) {
                for (int column = 0; column < rxB; column ++) {
                    if ( rawValuesB[row][column] < touchThresholdForTouchB[row][column] ) {
                        touchThresholdForTouchB[row][column] = rawValuesB[row][column];
                    }
                }
            }
//             为所有的点取最大值
//            for (int row = 0; row < txB; row ++) {
//                for (int column = 0; column < rxB; column ++) {
//                    if ( rawValuesB[row][column] > touchThresholdForTouchB[row][column] ) {
//                        touchThresholdForTouchB[row][column] = rawValuesB[row][column];
//                    }
//                }
//            }
        }

        if ( currentTimeMillis() - first_calibration_done_time > 20000 ) {
            touchBaseLineSetforB = true;
            System.out.println("触碰基线已经设置成功, 下方数组展示了SensorB所有点的触碰时的最小值。");
            parent.background(parent.color(0,0,0));
            parent.fill(255);
            parent.text("传感器已经全部校准完成，可以开始使用SensorA和SensorB传感器识别您的手势。", 20, 30);

            // 输出校准后的每个点的值
            for (int row = 0; row < txB; row ++) {
                for (int column = 0; column < rxB; column ++) {
                    System.out.printf("%5d ", touchThresholdForTouchB[row][column]);
                }
                System.out.println();
            }
            // 如果计算出得thresholdForTouchA[][]的某个点是负的或者是0，这个点被归位盲点。
            for (int row = 0; row < txB; row ++) {
                for (int column = 0; column < rxB; column ++) {
                    blindPointsB[row][column] = touchThresholdForTouchB[row][column] - maxNoiseValuesB[row][column];
                    if (blindPointsB[row][column] > 0) {
                        blindPointsB[row][column] = 1;
                    } else {
                        blindPointsB[row][column] = 0;
                    }
                }
            }
            // 计算每个点的阈值, 这里的一个关键阈值使用了0.5，设置的较小是因为比较容易检测到touch down
            for (int row = 0; row < txB; row ++) {
                for (int column = 0; column < rxB; column ++) {
                    thresholdForTouchB[row][column] = (int) (maxNoiseValuesB[row][column] + thresholdfortouchB*(touchThresholdForTouchB[row][column] - maxNoiseValuesB[row][column]));
                }
            }
            saveArrayToFile(touchThresholdForTouchB, "arrayTouchThresholdForTouchB");
            saveArrayToFile(blindPointsB, "arrayBlindPointsB");
            saveArrayToFile(thresholdForTouchB, "arrayThresholdForTouchB");
        }
    }

    // 将thresholdForTouchA[tx][rx] 与 blindPointsA[tx][rx] 存储为文件供isAB_touchdown()读取
    public void saveArrayToFile(int[][] array, String filename) {
        FileWriter writeFile = null; // 创建字符输出流
        try {
            File file = new File("/Users/cuizhitong/OneDrive - zju.edu.cn/FGS_2021/FGS_Code/Processing_Code/showMeSomething/"+filename+".txt"); // 数据想写入的路径及文件
            if(!file.exists()) { // 如果该文件不存在，就创建
                file.createNewFile();
            }
            writeFile = new FileWriter(file); // 给字节输出流赋予实例
            writeFile.write(""); // 首先需要清空文件的内容
            // 通过循环将数组写入txt文件中
            for(int i = 0; i < array.length; i++) {
                for(int j = 0; j < array[0].length - 1; j++) { // 数据前n - 1列尾部加入","
                    writeFile.write(array[i][j] + ",");
                }
                writeFile.write(array[i][array[0].length - 1] + ""); // 数组最后一列后面不加","
                writeFile.write("\n"); // 加上换行符
            }
            writeFile.flush(); // 把writeFile里的数据全部刷新一次，全部写入文件中
        } catch (Exception e) { // 异常捕获
            e.printStackTrace();
        } finally {
            try {
                if(writeFile != null) // 如果writeFile不为空，就将其关闭
                    writeFile.close();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }

    // 在计算SensorA与SensorB的手势时首先从文件中读取各个数组
    public int[][] readArrayFile(String filename) {
        FileReader reader = null; // 声明一个字符输入流
        BufferedReader readerBuf = null; // 声明一个字符输入缓冲流
        int[][] array = null; // 声明一个二维数组
        try {
            reader = new FileReader("/Users/cuizhitong/OneDrive - zju.edu.cn/FGS_2021/FGS_Code/Processing_Code/showMeSomething/"+filename+".txt"); // 指定reader的读取路径
            readerBuf = new BufferedReader(reader); // 通过BufferedReader包装字符输入流
            ArrayList<String> strList = new ArrayList<>(); // 创建一个集合，用来存放读取的文件的数据
            String lineStr; // 用来存放一行的数据
            while((lineStr = readerBuf.readLine()) != null) { // 逐行读取txt文件中的内容
                strList.add(lineStr); // 把读取的行添加到list中
            }
            int lineNum = strList.size(); // 获取文件有多少行
            String s =  strList.get(0); // 获取数组有多少列
            int columnNum = s.split("\\,").length;
            array = new int[strList.size()][columnNum]; // 根据文件行数创建对应的数组
            int count = 0; // 记录输出当前行
            for(String str : strList) { // 循环遍历集合，将集合中的数据放入数组中
                String[] strs = str.split("\\,"); // 将读取的str按照","分割，用字符串数组来接收
                for(int i = 0; i < columnNum; i++) {
                    array[count][i] = Integer.valueOf(strs[i]);
                }
                count++; // 将行数 + 1
            }
        } catch (Exception e) {
            e.printStackTrace();
        } finally {
            try { // 关闭字符输入流
                if(reader != null)
                    reader.close();
            } catch (IOException e) {
                e.printStackTrace();
            }
            try { // 关闭字符输入缓冲流
                if(readerBuf != null)
                    readerBuf.close();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
        return array; // 返回稀疏数组
    }

    // 设置SensorA的手势
    public void setSensorAGesture(int sensorAGesture, String gestureName) {
        // 这里的变量重置非常关键
        tapCount = 0;
        touchDurationA = 0;
        touchDistanceA = 0;
        if (sensorAGesture != A_SLIDE_UP && sensorAGesture != A_SLIDE_DOWN && sensorAGesture != A_SLIDE_LEFT && sensorAGesture != A_SLIDE_RIGHT) {
            maxdistance = 0; // touchDistanceA置为0的同时，maxdistance也需要置为0，但是这里需要排除slide手势
        }
        this.sensorAGesture = sensorAGesture;
        // 在Processing中展示当前的手势
//        parent.fill(255);
//        parent.textSize(48);
//        if (gestureName != "NO_GESTURE") {
//            lastSensorAGestureName = gestureName;
//        }
//        parent.text("SensorA: " + lastSensorAGestureName, 20, 800);
    }

    // 设置SensorB的手势
    public void setSensorBGesture(int sensorBGesture, String gestureName) {
        // 这里的变量重置非常关键
        tapCountB = 0;
        touchDurationB = 0;
        touchDistanceB = 0;
        this.sensorBGesture = sensorBGesture;
        parent.fill(255);
        parent.textSize(48);
        if (gestureName != "B_NO_GESTURE") {
            lastSensorBGestureName = gestureName;
        }
        parent.text("SensorB: " + lastSensorBGestureName, 800, 800);
    }

    // 通过点阵与map映射的方式视觉化此时的视觉情况
    public void showMeSomething() {
        int recLength = 60;
        float adjustValueA;
        float adjustValueB;
        if (readAdata) { // 展示SensorB
            for(int row = 0; row < txA; row++) {
                for(int column = 0; column < rxA; column++) {
//                if (outputCount == 0) {starting_time_forOutputCount = currentTimeMillis();}
//                outputCount++;
                    adjustValueA = parent.map(rawValuesA[row][column], arrayMaxNoiseValuesA[row][column], arrayTouchThresholdForTouchA[row][column], 0, 255); // fill adjusted values
                    parent.fill(0, 255, 255, (int) adjustValueA);
                    parent.rect(620 - recLength * row, 10 + recLength * column, recLength, recLength);
                }
            }
        }
        if (readBdata) { // 展示SensorA
            for(int row = 0; row < txB; row++) {
                for(int column = 0; column < rxB; column++) {
                    adjustValueB = parent.map(rawValuesB[row][column], arrayMaxNoiseValuesB[row][column], arrayTouchThresholdForTouchB[row][column], 0, 255); // fill adjusted values
                    parent.fill(0, 255, 255, (int) adjustValueB);
                    parent.rect(800 + recLength * column, 10 + recLength * row, recLength, recLength);
                }
            }
        }
//        if (currentTimeMillis() - starting_time_forOutputCount > 5000) {
//            float durationBySeconds = (float)(currentTimeMillis() - starting_time_forOutputCount) / 1000;
//            int count_rate = (int) (outputCount / durationBySeconds / (txA*rxA));
//            System.out.println("传感器的A输出帧率是: " + count_rate);
//            outputCount = 0;
//        }
    }

    // Processing获取SensorA的手势
    public int getSensorAGesture() {
        return sensorAGesture;
    }

    // Processing获取SensorB的手势
    public int getSensorBGesture() {
        return sensorBGesture;
    }

    // 连接串口
    public void connect(int serialPort) {
        if (serialPort <= Serial.list().length) {
            myPort = new Serial(parent, Serial.list()[serialPort], 115200);
            //clear the buffer
            myPort.clear();
            //always buffer until the newline symbol, ASCII linefeed
            myPort.bufferUntil(10);
            connected = true;
            System.out.println("Connected Successfully!!!");
        } else {
            System.out.println("Error: serial port not found, please reset the number of serialport");
            connected = false;
        }
    }

    // 这个readSerial()读取两个传感器的原始数值
    public void readSerial() {
        changesbyA = false;
        changesbyB = false;

        if (connected) {
            while (myPort.available() > 0) {
                String myString = myPort.readStringUntil('\n');

                // 在 myString == null 时break非常关键，可以有效的提高帧率，因为非常一大部分 myString == null，耗费了大量的运行时间, 导致了空循环
                // 设置这个break之后，帧率显著提高到 60fps
                if (myString == null) { break; }

                // Data Structure: Sensor A and Sensor B
                if (myString != null) {
                    // remove unnecessary parts of the string (potential corrupted parts too)
                    myString = myString.replaceAll("[^AB0-9,]+", "");
                    if (!myString.contains(",")) { break; }
//                    System.out.println(myString); // 在processing中检查所收到的电压值

                    frame_count += 1;
                    if (frame_count == 1) { starting_time = currentTimeMillis(); } // set the system start_time, from the first string received
                    // calculate the framerate of Sensor A
                    // calcFrameRate();

                    // Read signals from Sensor A and Sensor B
                    if (myString.charAt(0) == 'A') {
                        myString = myString.substring(1); // remove the first char 'A'
                        myString = myString.substring(1); // remove the first char ','
                        String[] splitString = myString.split(","); // Split the string into rx+1 numbers (still string type though)
                        if (splitString.length != this.rxA + 1) { break; } // Check if the correct amount of strings are in the array
                        if (!splitString[0].matches("[0-9]+")) { break; }
                        int transmit = Integer.parseInt(splitString[0]); // TX of the received input
                        // Save array rawValuesA[][]
                        for (int i = 1; i < splitString.length; i++) {
                            changesbyA = true;
                            rawValuesA[transmit][i-1] = Integer.parseInt(splitString[i]);
                        }
                    } else if (myString.charAt(0) == 'B') { // Read signals from sensor B
                        myString = myString.substring(1); // remove the first char 'B'
                        myString = myString.substring(1); // remove the first char ','
                        String[] splitString = myString.split(",");
                        if (splitString.length != this.rxB + 1) { break; }
                        if (!splitString[0].matches("[0-9]+")) { break; }
                        int transmit = Integer.parseInt(splitString[0]); // TX of the received input
                        // Save array rawValuesB[][]
                        for (int i = 1; i < splitString.length; i++) {
                            changesbyB = true;
                            rawValuesB[transmit][i-1] = Integer.parseInt(splitString[i]);
                        }
                    }
                }
            }
        } else {
            System.out.println("没有收到信号，请检查硬件连接设置。");
        }
    }

    // 计算帧率fps
    public void calcFrameRate() {
        // 这里的计算有些数据格式转换的问题，可能需要重写此函数
        if (frame_count >= 1000) {
            float durationBySeconds =  ((float)(currentTimeMillis() - starting_time) / 1000);
            int frame_rate = (int) (frame_count / durationBySeconds / txA);
            System.out.println("传感器的A接收帧率是: " + frame_rate);
            frame_count = 0;
        }
    }

    // 这个readSerial()只读取一个传感器的值
//    public void readSerial() {
//        changesbyA = false;
//        changesbyB = false;
//
//        if (connected) {
//            while (myPort.available() > 0) {
//                String myString = myPort.readStringUntil('\n');
//
//                // 在 myString == null 时break非常关键，可以有效的提高帧率，因为非常一大部分 myString == null，耗费了大量的运行时间, 导致了空循环
//                // 设置这个break之后，帧率显著提高到 60fps
//                if (myString == null) { break; }
//
//                // Data Structure: Sensor A and Sensor B
//                if (myString != null) {
//                    // remove unnecessary parts of the string (potential corrupted parts too)
//                    myString = myString.replaceAll("[^AB0-9,]+", "");
//                    if (!myString.contains(",")) {
//                        break;
//                    }
////                    System.out.println(myString); // 在processing中检查所收到的电压值
//
//                    // set system start_time
//                    frame_count += 1;
//                    if (frame_count == 1) { starting_time = currentTimeMillis(); }
//                    // calculate the framerate of Sensor A
////                    calc_frameRate();
//
//                    // Read signals from Sensor A and Sensor B, remove the first char 'A', if (myString.charAt(0) == 'A')
//                    // myString = myString.substring(1);
//
//                    // Split the string into rx+1 numbers (still string type though)
//                    String[] splitString = myString.split(",");
//                    // Check if the correct amount of strings are in the array
//                    if (splitString.length != this.rxA + 1) {
//                        break;
//                    }
//                    if (!splitString[0].matches("[0-9]+")) {
//                        break;
//                    }
//                    // TX of the received input
//                    int transmit = Integer.parseInt(splitString[0]);
//
//                    // Save the new rawValuesA
//                    // println(splitString);
//                    for (int i = 1; i < splitString.length; i++) {
//                        changesbyA = true;
//                        rawValuesA[transmit][i - 1] = Integer.parseInt(splitString[i]);
//                    }
//
//                    // Read signals from sensor B
//                }
//            }
//        } else {
//            System.out.println("没有收到信号，请检查硬件连接设置。");
//        }
//    }

} // the end