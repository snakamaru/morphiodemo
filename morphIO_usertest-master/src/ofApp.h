#pragma once

#include "ofMain.h"
#include "ofxGui.h"
#include "ofxHistoryPlot.h"

class ofApp : public ofBaseApp{
    
public:
    void setup();
    void update();
    void draw();
    void keyPressed(int key);
    void keyReleased(int key);
    void mouseMoved(int x, int y );
    void mouseDragged(int x, int y, int button);
    void mousePressed(int x, int y, int button);
    void mouseReleased(int x, int y, int button);
    void mouseEntered(int x, int y);
    void mouseExited(int x, int y);
    void windowResized(int w, int h);
    void dragEvent(ofDragInfo dragInfo);
    void gotMessage(ofMessage msg);
    
    /*-----input-----*/
    static const int ANALOG_NUM = 1;//モジュールの個数
    static const int TOTAL_ANALOG_NUM = ANALOG_NUM * 2;
    
    int analogPinNum[TOTAL_ANALOG_NUM] = {0};
    int outputGPIO = ANALOG_NUM * 2;
    int analogNumStart[2] = {0,2};
    
    int analogEx[2] = {0};
    
    /*-----output-----*/
    
    int valveNumStart = 14;
    int supplyValve[ANALOG_NUM] = {14};
    int vacuumValve[ANALOG_NUM] = {15};
    
    int pumpNumStart = 2;
    int supplyPump[ANALOG_NUM] = {3};
    int vacuumPump[ANALOG_NUM] = {5};
    
    int pwm = 180;
    int ext = 20;
    int propEx[ANALOG_NUM] = {0};
    int mappingEx(int number);
    
    /*-----setup-----*/
    static const int DEFORM_RESOLUSION = 100;
    static const int FRAMERATE_NUM = 60; //1sec
    static const int RECORD_NUM = FRAMERATE_NUM * 8; //nSec
    
    static const int MIDDLE_VALUE = 600;
    int THRESHOLD_VALUE = 100;
    int minValue[TOTAL_ANALOG_NUM] = {MIDDLE_VALUE};
    int maxValue[TOTAL_ANALOG_NUM] = {MIDDLE_VALUE};
    int manipulateInput, manipulateOutput;
    
    /*-----------------------------------------------------------------*/
    
private:
    void workRecord();
    void workPlay();
    void workRealtime();
    
    void countClear();
    
    void ffJudge(int number);
    void fbJudge(int number);
    void ffOutput(int number, int input);
    void fbOutput(int number);
    void checkOutput(int x);
    
    bool bRecord = false;
    bool bPlay = false;
    bool bReal = false;
    bool bCheck = false;
    
    void checkWrite();
    void useImportData();
    void captureScreen();
    
    //output
    bool bDeform[ANALOG_NUM] = {false};
    bool bPolarity[ANALOG_NUM] = {false};
    int forClosedLoopDelta[ANALOG_NUM] = {0};
    int oldDelta = 0;
    float startTime = 0;
    float milliSeconds = 0;
    
    bool bThreshold[TOTAL_ANALOG_NUM] = {false};
    bool bNeutral[ANALOG_NUM] = {false};
    int neutralVal = 50;
    int neuThreshold = 5;
    
    
    void stopActuate(int number);
    void coolDown();
    void checktime();
    
    //PID
    int delta[TOTAL_ANALOG_NUM][2] = {{0},{0}};
    int absDelta[TOTAL_ANALOG_NUM] = {0};
    float dt = 1 / FRAMERATE_NUM;//0.05
    float integral;
    float KP = 0.0; //Pゲイン
    float KI = 0.0; //Iゲイン
    float KD = 0.1; //Dゲイン
    float p = 0.0;
    float i = 0.0;
    float d = 0.0;
    
    int PWM[ANALOG_NUM] = {0};
    int setPWM_PID(int p, int i, int d, int number);
    
    void sendDigitalArduinoSupply(int number, int PWM);
    void sendDigitalArduinoVacuum(int number, int PWM);
    void sendDigitalArduinoClose(int number);
    void sendDigitalArduinoExhaust(int number);
    
    void checkDigital(int number);
    void clearDigital();
    void checkPWM(int number, int PWM);
    
    int count = 0;
    void actuate(int number, int deltaThreshold);
    
    int playCount = 0;
    
    ofArduino ard; //arduino
    bool bSetupArduino;
    void initArduino();
    void setupArduino(const int & version);
    void digitalPinChanged(const int & pinNum);
    void analogPinChanged(const int & pinNum);
    void updateArduino();
    
    string buttonState;
    string potValue;
    
    int filteredValue[ANALOG_NUM][2] = {0};
    int propVol[ANALOG_NUM] = {0};
    int oldPropVol[ANALOG_NUM] = {0};
    int recordPropVol[ANALOG_NUM][RECORD_NUM] = {0};
    
    void adjustAnalog(int pin, int order);
    void adjustAnalogEx(int pin, int order);
    
    void updateVal(int order);
    
    double ceil2(double dIn, int nLen);
    float a = 0.9;
    
    bool bRelay = false;
    
    //graphics
    float width = ofGetWidth()/2;
    float height = ofGetHeight()/2;
    ofTrueTypeFont      font;
    ofTrueTypeFont      smallFont;
    float valueRow[3] = {20, width * 2 - 200, width * 2 - 350};
    float valueCol[3] = {250, 400, 550};
    
    float contentWidth = 200;
    float row[2][3] = {{20, 20 + contentWidth, 20 + contentWidth * 2}, {width, width + contentWidth, width + contentWidth * 2}};
    int column = 700;
    void drawLog();
    void drawLogContents(int number, int row, int order);
    void drawArrayData(int number);
    
    ofFile recordFile;
    ofFile feedbackFile;
    bool bRecordWrite = false;
    bool bPlayWrite = false;
    
    //------feedback-------
    
    
    ofImage screen;
    
    //offile
    ofFile myTextFile[ANALOG_NUM];
    
    //ofxgui
    ofxPanel gui;
    ofxFloatSlider operateMax[ANALOG_NUM];
    ofxFloatSlider operateMin[ANALOG_NUM];
    
    ofxIntSlider pGain;
    ofxFloatSlider vGain;
    ofxIntSlider threshold;
    
    //plot
    ofxHistoryPlot * plot[ANALOG_NUM];
    ofxHistoryPlot * recordPlot[ANALOG_NUM];
    void setupHistoryPlot(int number);
    
    float currentFrameRate;
    bool bDrawPlot = false;
    
    bool bActive[3] = {false};
};
