#include "ofApp.h"
void ofApp::setup(){
    ofBackground(20, 20, 20);
    ofSetVerticalSync(true);
    ofSetFrameRate(FRAMERATE_NUM);
    font.load("font/franklinGothic.otf", 10);
    smallFont.load("font/franklinGothic.otf", 10);
    initArduino();
    gui.setup();
    gui.add(pGain.setup("pGain", 8, 0, 10));
    gui.add(vGain.setup("vGain", 0.5, 0.0, 1.0));
    gui.add(threshold.setup("threshold", 2, 0, 50));
    
    /*-----------input init-------*/
    for (int i = 0; i< ANALOG_NUM; i++){
        analogPinNum[i] = {i}; //A0~2
        
        gui.add(operateMax[i].setup("minValue: A" + ofToString(analogPinNum[i]),MIDDLE_VALUE - THRESHOLD_VALUE, 0, 1023));
        gui.add(operateMin[i].setup("MaxValue: A" +   ofToString(analogPinNum[i]),MIDDLE_VALUE + THRESHOLD_VALUE, 0, 1023));
        
        setupHistoryPlot(i);
    }
    
    /*---------output init----*/
    for (int i = valveNumStart; i < outputGPIO + valveNumStart; i++) {
        ard.sendDigital(i, ARD_LOW);
    }
    
    ard.sendPwm(3, 0);
    ard.sendPwm(5, 0);
    ard.sendPwm(6, 0);
    ard.sendPwm(9, 0);
    ard.sendPwm(10, 0);
    ard.sendPwm(11, 0);
}

void ofApp::update(){
    currentFrameRate = ofGetFrameRate();
    ard.update();
    
    //EX使うために
    for (int i = 0; i < ANALOG_NUM; i++) {
        adjustAnalog(analogPinNum[i], i);
        plot[i]->update(propVol[i]);
        //        recordPlot[i]->update(recordPropVol[i][count]);
    }
    
    //    for (int i = 0; i < ANALOG_NUM; i++) {
    //        //recordPlot[i]->update(recordPropVol[i][count]);
    //    }
    
    workRecord();
    workPlay();
    //checkWrite();
    
}

//------------------------------------------------------------

void ofApp::adjustAnalog(int _pin, int _order){
    
    filteredValue[_order][1] = a * filteredValue[_order][0] + (1-a) * ard.getAnalog(_order);
    
    propVol[_order] = ofMap(filteredValue[_order][1], minValue[_order], maxValue[_order], DEFORM_RESOLUSION, 0);
    
    if(filteredValue[_order][1] > maxValue[_order]){
        maxValue[_order] = filteredValue[_order][1];
    }
    if(filteredValue[_order][1] < minValue[_order]){
        minValue[_order] = filteredValue[_order][1];
    }
}

void ofApp::adjustAnalogEx(int _pin, int _order){
    filteredValue[_order][1] = a * filteredValue[_order][0] + (1-a) * ard.getAnalog(_order);
    
    propVol[_order] = ofMap(filteredValue[_order][1], minValue[_order], maxValue[_order], DEFORM_RESOLUSION + ext, 0 - ext);
    
    if(filteredValue[_order][1] > maxValue[_order]){
        maxValue[_order] = filteredValue[_order][1];
    }
    if(filteredValue[_order][1] < minValue[_order]){
        minValue[_order] = filteredValue[_order][1];
    }
}

int ofApp::mappingEx(int number){
    propEx[number] = ofMap(propVol[number],DEFORM_RESOLUSION, 0, DEFORM_RESOLUSION + ext, 0 - ext);
    return propEx[number];
}


void ofApp::workRecord(){
    if (bRecord == true) {
        if (count == RECORD_NUM) {
            bRecord = false;
            countClear();
            bRecordWrite = true;
        } else {
            for (int i = 0; i < ANALOG_NUM; i++) {
                mappingEx(i);
                recordPropVol[i][count] = propEx[i];
            }
        }
        count++;
    }
}

void ofApp::checkWrite(){
    if (bRecordWrite == true) {
        for(int i = 0; i < ANALOG_NUM; i++){
            myTextFile[i].open(ofGetTimestampString() + " (" + ofToString(i)+ ") " + "text.txt",ofFile::WriteOnly);
            for(int j = 0;j < RECORD_NUM;j++){
                myTextFile[i] << j
                << ": " << recordPropVol[i][j] << endl;
            }
        }
        bRecordWrite = false;
    }
}

void ofApp::workPlay(){
    if (bPlay == true) {
        
        std::cout << "propVol[0]" + ofToString(propVol[0]) + "propEx[0]" + ofToString(propEx[0]) << endl;
        for (int i = 0; i < ANALOG_NUM; i++) {
            recordPlot[i]->update(recordPropVol[i][playCount]);
        }
        
        if (playCount == RECORD_NUM) {
            bPlay = false;
            countClear();
            for (int i = 0; i < ANALOG_NUM; i++) {
                sendDigitalArduinoExhaust(i);
            }
        } else {
            for (int i = 0; i < ANALOG_NUM; i++) {
                fbJudge(i);
                fbOutput(i);
            }
        }
        playCount++;
        
        //        if (playLoop < LOOP_TIME - 1) {
        //            if (playCount == RECORD_NUM) {
        //                playLoop++;
        //                playCount = 0;
        //            }
        //        }
    }
}

void ofApp::workRealtime(){
    //    //turn on press'p', turn off press'o'.
    //    if (bReal == true) {
    //        for (int i = 3; i < 6; i++) {
    //            fbJudge(i, i-3);
    //            fbOutput(i-3,i);
    //        }
    //    } else {
    //        for (int i = 0; i < ANALOG_NUM; i++) {
    //            sendDigitalArduinoExhaust(i);
    //        }
    //    }
}

void ofApp::countClear(){
    count = 0;
    playCount = 0;
}


/*--------feedback------------*/

void ofApp::fbJudge(int number){ //目標値，センサー値
    
    delta[number][0] = delta[number][1]; //0が過去
    delta[number][1] = recordPropVol[number][playCount] - propVol[number]; //偏差の取得
    absDelta[number] = abs(delta[number][1]); //偏差の絶対値
    
    if(absDelta[number] >= threshold) {
        bDeform[number] = true;
    } else {
        bDeform[number] = false;
    }
    
    if(absDelta[number] <= 10) {
        bThreshold[number] = true;
    } else {
        bThreshold[number] = false;
    }
    
    if (delta[number][1] > 0) {
        bPolarity[number] = true;
        KP = pGain;
    } else if (delta[number][1] < 0){
        bPolarity[number] = false;
        KP = pGain * vGain;
    }
    
    if (neutralVal - neuThreshold < propVol[number] && propVol[number] < neutralVal) { //45~ 55のとき
        bNeutral[number] = true;
    } else {
        bNeutral[number] = false;
    }
    
    
    p = KP * delta[number][1]; //定数*偏差(0~100) 255 =  gain * 100
    i = KI * integral;
    d = KD * ((delta[number][1] - delta[number][0]) / dt);
    
    setPWM_PID(p, 0, 0, number);
}

int ofApp::setPWM_PID(int p, int i, int d, int number){
    PWM[number] = abs(p + i + d);
    if(PWM[number] > 255){
        PWM[number] = 255;
    } else if (PWM[number]<50){
        PWM[number] = 0;
    }
    return PWM[number];
}

void ofApp::fbOutput(int number){
    //    if (bNeutral[number]) {
    //        sendDigitalArduinoExhaust(number);
    //    } else {
    if (bDeform[number] == true) {
        if (bPolarity[number] == true) {
            sendDigitalArduinoSupply(number, PWM[number]);
            std::cout << ofToString(number) + ": supply" << endl;
        } else {
            sendDigitalArduinoVacuum(number, PWM[number]);
            std::cout << ofToString(number) + ": vacuum" << endl;
        }
    } else {
        sendDigitalArduinoClose(number);
        std::cout << ofToString(number) + ": close" << endl;
    }
    //}
}

/*----------------arduino control------------*/
void ofApp::sendDigitalArduinoSupply(int number, int PWM){
    ard.sendDigital(supplyValve[number], ARD_HIGH);
    ard.sendDigital(vacuumValve[number], ARD_LOW);
    ard.sendPwm(supplyPump[number], PWM);
    ard.sendPwm(vacuumPump[number], 0);
}

void ofApp::sendDigitalArduinoVacuum(int number, int PWM){
    ard.sendDigital(supplyValve[number], ARD_LOW);
    ard.sendDigital(vacuumValve[number], ARD_HIGH);
    ard.sendPwm(supplyPump[number], 0);
    ard.sendPwm(vacuumPump[number], PWM);
}

void ofApp::sendDigitalArduinoClose(int number){
    ard.sendDigital(supplyValve[number], ARD_HIGH);
    ard.sendDigital(vacuumValve[number], ARD_LOW);
    ard.sendPwm(supplyPump[number], 0);
    ard.sendPwm(vacuumPump[number], 0);
}

void ofApp::sendDigitalArduinoExhaust(int number){
    ard.sendDigital(supplyValve[number], ARD_LOW);
    ard.sendDigital(vacuumValve[number], ARD_LOW);
    ard.sendPwm(supplyPump[number], 0);
    ard.sendPwm(vacuumPump[number], 0);
}

void ofApp::checkOutput(int x) {
    if (bCheck == true) {
        if (x == 1) {
            sendDigitalArduinoVacuum(0,0);
        }
    } else {
        sendDigitalArduinoExhaust(0);
    }
}
//*****--------Draw--------------------****//

void ofApp::draw(){
    gui.draw();
    drawLog();
    
    for (int i = 0; i <ANALOG_NUM; i++) {
        plot[i]->draw(0, 110 * (i+1) + 150, ofGetWidth(), 100);
    }
    
    //plot[2]->draw(0, 110 * (2) + 150, ofGetWidth(), 100);
    
    if(bPlay == true){
        for (int i = 0; i <ANALOG_NUM; i++) {
            recordPlot[i]->draw(0, 110 * (i+1) + 150, ofGetWidth(), 100);
        }
        //recordPlot[2]->draw(0, 110 * (2) + 150, ofGetWidth(), 100);
    }
    
    
    for (int i = 0; i < ANALOG_NUM; i++) {
        updateVal(i);
        oldPropVol[i] = propVol[i];
    }
}


void ofApp::updateVal(int _order){
    filteredValue[_order][0] = filteredValue[_order][1];
}

void ofApp::drawLog(){
    ofSetColor(255);
    
    if (!bSetupArduino){
        font.drawString("Connect ready...\n", valueRow[1], 30);
    } else {
        if (bRecord == true) {
            font.drawString("Recording...\n", valueRow[1], 30);
        } else if (bPlay == true) {
            font.drawString("Playing...\n", valueRow[1], 30);
        } else {
            font.drawString("Connect succeed!\n", valueRow[1], 30);
        }
    }
    smallFont.drawString("bRecord : bPlay" + ofToString(bRecord) + " : " + ofToString(bPlay) , valueRow[1], 50);
    smallFont.drawString("count" + ofToString(count), valueRow[1], 150);
    smallFont.drawString("playCount" + ofToString(playCount), valueRow[1], 170);
    
    smallFont.drawString("propVol" + ofToString(propVol[0]), valueRow[1], 190);
    smallFont.drawString("bPorality" + ofToString(bPolarity[0]), valueRow[1], 210);
    
    ofSetColor(255);
    for (int i = 0; i < ANALOG_NUM; i++) {
        drawLogContents(i, 0, i);
    }
}

void ofApp::drawLogContents(int _number, int _row, int _order){
    font.drawString("Propotion : " + ofToString(propVol[_number]), row[_row][_order], 700);
    smallFont.drawString("value  :  " + ofToString(filteredValue[_number][1]), row[_row][_order], 715);
    smallFont.drawString("minValue  :  " + ofToString(minValue[_number]), row[_row][_order], 730);
    smallFont.drawString("maxValue     :  " + ofToString(maxValue[_number]), row[_row][_order], 745);
    smallFont.drawString("delta : " + ofToString(propVol[_number] - oldPropVol[_number]), row[_row][_order], 760);
}

void ofApp::useImportData(){
    //------import file--------
//    myReadFile.open("text.txt",ofFile::ReadOnly);
//    cout << myReadFile.readToBuffer().getText();
//    auto input = ofSplitString(myReadFile.readToBuffer().getText(), "\n");
//    for(int i= 0; i < RECORD_NUM;i++)
//    {
//        recordPropVol[i] = stoi(input[i]);
//    }
    
    //-------export file------------------
    recordFile.open(ofGetTimestampString() + "recordData.txt",ofFile::WriteOnly);
    feedbackFile.open(ofGetTimestampString() + "feedBackData.txt",ofFile::WriteOnly);
    
}

void ofApp::captureScreen(){
    //スクリーンのキャプチャをとる
    //    feedbackFile << propotionVolume[0]  << endl;
    //    screen.grabScreen(0, 0 , ofGetWidth(), ofGetHeight());
    //    screen.save(ofGetTimestampString() + "screenshot.png");
}


/*-----Default oF------*/

void ofApp::keyPressed(int key){
    switch (key) {
        case 'f':
            ofToggleFullscreen();
            break;
        case 'c':
            for (int i = 0; i < ANALOG_NUM; i++) {
                minValue[i] = operateMin[i];
                maxValue[i] = operateMax[i];
            }
            break;
        case 'v':
            milliSeconds = 0;
            break;
        case 's':
            bRecord = true;
            //countClear();
            break;
        case 'r':
            bPlay = true;
            for (int i = 0; i < ANALOG_NUM; i++) {
                plot[i]->reset();
                recordPlot[i]->reset();
            }
            break;
        case 'p':
            bReal = true;
            break;
        case 'o':
            bReal = false;
            break;
        case 'w':
            //bRecordWrite = true;
            break;
        case 't':
            bCheck = true;
            break;
        case  'y':
            bCheck = false;
            break;
        case '1':
            checkPWM(3, 100);
            break;
        case '2':
            checkPWM(3, 0);
            break;
        case '3':
            checkDigital(16);
            break;
        case '4':
            checkDigital(17);
            break;
        case '5':
            checkDigital(14);
            break;
        case '6':
            checkDigital(15);
            break;
        case '7':
            checkDigital(18);
            break;
        case '8':
            checkDigital(19);
            break;
        case '0':
            clearDigital();
            break;
        default:
            break;
    }
}

void ofApp::checkDigital(int number){
    ard.sendDigital(number, ARD_HIGH);
}

void ofApp::checkPWM(int number, int PWM) {
    ard.sendPwm(number, PWM);
}

void ofApp::clearDigital(){
    for (int i = 0; i < 20; i++) {
        ard.sendDigital(i, ARD_LOW);
    }
}

void ofApp::keyReleased(int key){
    switch (key) {
        default:
            break;
    }
}
void ofApp::mouseMoved(int x, int y ){
}
void ofApp::mouseDragged(int x, int y, int button){
}
void ofApp::mousePressed(int x, int y, int button){
}
void ofApp::mouseReleased(int x, int y, int button){
}
void ofApp::mouseEntered(int x, int y){
}
void ofApp::mouseExited(int x, int y){
}
void ofApp::windowResized(int w, int h){
}
void ofApp::gotMessage(ofMessage msg){
}
void ofApp::dragEvent(ofDragInfo dragInfo){
}

void ofApp::initArduino(){
    ard.connect("/dev/cu.usbmodem14101", 57600);
    
    ofAddListener(ard.EInitialized, this, &ofApp::setupArduino);
    bSetupArduino    = false;
}

void ofApp::setupArduino(const int & version) {
    ofRemoveListener(ard.EInitialized, this, &ofApp::setupArduino);
    bSetupArduino = true;
    ofLogNotice() << ard.getFirmwareName();
    ofLogNotice() << "firmata v" << ard.getMajorFirmwareVersion() << "." << ard.getMinorFirmwareVersion();
    
    //A0~5
    for (int i = analogNumStart[0]; i < TOTAL_ANALOG_NUM; i++) {
        ard.sendAnalogPinReporting(i, ARD_ANALOG);
    }
    
    //14~19 for the valve
    for (int i = valveNumStart; i < valveNumStart + outputGPIO; i++) {
        ard.sendDigitalPinMode(i, ARD_OUTPUT);
    }
    
    //PWMの設定
    ard.sendDigitalPinMode(3, ARD_PWM);
    ard.sendDigitalPinMode(5, ARD_PWM);
    ard.sendDigitalPinMode(6, ARD_PWM);
    ard.sendDigitalPinMode(9, ARD_PWM);
    ard.sendDigitalPinMode(10, ARD_PWM);
    ard.sendDigitalPinMode(11, ARD_PWM);
    
    ofAddListener(ard.EDigitalPinChanged, this, &ofApp::digitalPinChanged);
    ofAddListener(ard.EAnalogPinChanged, this, &ofApp::analogPinChanged);
}

void ofApp::digitalPinChanged(const int & pinNum) {
    // do something with the digital input. here we're simply going to print the pin number and
    // value to the screen each time it changes
    buttonState = "digital pin: " + ofToString(pinNum) + " = " + ofToString(ard.getDigital(pinNum));
}

void ofApp::analogPinChanged(const int & pinNum) {
    // do something with the analog input. here we're simply going to print the pin number and
    // value to the screen each time it changes
    potValue = "analog pin: " + ofToString(pinNum) + " = " + ofToString(ard.getAnalog(pinNum));
}

void ofApp::setupHistoryPlot(int number){
    plot[number] = new ofxHistoryPlot(&currentFrameRate, "number:00" + ofToString(number), ofGetWidth(), false);
    plot[number]->setBackgroundColor(ofColor(0,0,0,0));
    plot[number]->setColor(ofColor(255));
    plot[number]->setRange(-10, DEFORM_RESOLUSION /2 + 10);
    plot[number]->setRespectBorders(false);
    plot[number]->setLineWidth(2);
    plot[number]->setCropToRect(false);
    plot[number]->setShowSmoothedCurve(false);
    plot[number]->setSmoothFilter(0.1);
    
    recordPlot[number] = new ofxHistoryPlot(&currentFrameRate, "number:00" + ofToString(number), ofGetWidth(), false);
    recordPlot[number]->setBackgroundColor(ofColor(0,0,0,0));
    recordPlot[number]->setColor(ofColor(255, 0, 0));
    recordPlot[number]->setRange(-10, DEFORM_RESOLUSION /2 + 10);
    recordPlot[number]->setRespectBorders(false);
    recordPlot[number]->setLineWidth(2);
    recordPlot[number]->setCropToRect(false);
    recordPlot[number]->setShowSmoothedCurve(false);
    recordPlot[number]->setSmoothFilter(0.1);
}

double ofApp::ceil2(double dIn, int nLen){
    double dOut;
    dOut = dIn * pow(10.0, nLen);
    dOut = (double)(int)(dOut + 0.9);
    return dOut * pow(10.0, -nLen);
}
