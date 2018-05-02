#include "ofxDobot.h"


ofxDobot::ofxDobot(){

	noConnection = "Dobot not connected";

}

ofxDobot::~ofxDobot(){
    
    stopThread();
    
}

bool ofxDobot::setup(string serialName) {

	this->serialName = serialName;

	connected = serial.setup(serialName, 115200);

	waitingMessage = false;

	serial.flush();

	queuedCmdIndex = 0;
	queuedLeftSpace = 32;
	timeMessage = ofGetElapsedTimeMillis();
	currentFase = Begin;
    cmdTimeout = 1000;
    
    automaticUpdatePose = false;
    
    typeFileOpened = NONE;
    
    autoZ = 0;
    
    startThread();

	isStopped = true;
    
	return connected;

}




bool ofxDobot::load(string fileName) {

	cout << " file name = " << fileName << endl;
	ofFile file;
	bool exist = file.open(ofToDataPath(fileName));
	if ( exist ){
		if (file.getExtension() == "xml") {
			file.close();
            typeFileOpened = XML;
			if (timeline.loadFile(ofToDataPath(fileName))) {
				timeline.pushTag("root");
				rowIndex = 0;
			}

			else {
				ofLog(OF_LOG_ERROR, "file " + fileName + " not found");
			}

		}
		else if (file.getExtension() == "txt") {

			rowIndex = 0;
            typeFileOpened = TXT;

			buffer = file.readToBuffer();
			string textFile = buffer.getText();
			lines = ofSplitString(textFile, "\n");

		}
        else if ( file.getExtension() == "svg"){
            typeFileOpened = SVG;
            loadSVG(fileName);
        }
		else{
            typeFileOpened = NONE;
			ofLog(OF_LOG_ERROR,fileName +" extension not compatible");
		}
    }
    else{
        ofLog(OF_LOG_ERROR,fileName +" don't exist");
    }

    return exist;

}

void ofxDobot::setPolylines(vector<ofPolyline *> _polylines, int w, int h){
    
    
    currentPath = 0;
    currentCommand = 0;
    totalNumPoints = 0;
    currentPoint = 0;
    polylines = _polylines;
    typeFileOpened = SVG;
    
    for ( int i=0; i < polylines.size(); i++){
        totalNumPoints += polylines[i]->size();
    }
    
    widthDrawing = w;
    heightDrawing = h;
    
	CPCmd cmd;
	cmd.cpMode = 1;

	ofPoint to = convertToDobotCoordinate(polylines[0]->getVertices()[0]);
	cmd.x = to.x;
	cmd.y = to.y;
	cmd.z = 45;

	setCPCmd(cmd);
}

void ofxDobot::loadSVG(string fileName){
    
    svg.load(fileName);
    currentPath = 0;
    currentCommand = 0;
    totalNumPoints = 0;
    currentPoint = 0;
	printingCompletation = 0;
    cout << svg.getWidth() << " " << svg.getHeight() << endl;
    
    widthDrawing = svg.getWidth();
    heightDrawing = svg.getHeight();
    
    for ( int i=0; i < svg.getNumPath(); i++ ){
        ofPolyline *polyline = new ofPolyline();
        //ofPath path= ;
       // path.scale(DOBOT_XMAX-DOBOT_XMIN, DOBOT_XMAX-DOBOT_XMIN);
        //path.translate(ofPoint(100,100));
        *polyline = svg.getPathAt(i).getOutline()[0];
        polyline->simplify();
        polylines.push_back(polyline);
        cout << "num line of " << i << " polyline = " << polyline->size() << endl;
        totalNumPoints+=polyline->size();
    }
    
   
    
}

void ofxDobot::drawSVG(){
    
    svg.draw();
}

ofPoint ofxDobot::convertToDobotCoordinate(ofPoint p){
    
    ofPoint converted;
  
    converted.y = - (DOBOT_XMAX- DOBOT_XMIN ) / 2 +  ( p.x / widthDrawing )  * ( DOBOT_XMAX - DOBOT_XMIN );
    converted.x = DOBOT_XMIN + ( p.y / heightDrawing ) * ( DOBOT_XMAX - DOBOT_XMIN );
    
    return converted;
    
    
}

void ofxDobot::restart(){

	rowIndex = 0;
    currentPath = 0;
    currentCommand = 0;

}

void ofxDobot::play() {

	setQueuedCmdStartExec();
	isStopped = false;

}

void ofxDobot::stop() {

	setQueuedCmdStopExec();
	isStopped = true;

}

void ofxDobot::clear() {

	//clearAllAlarmsState();

	setQueuedCmdClear();
	isStopped = true;
	typeFileOpened = NONE;

	rowIndex = 0;
	currentPath = 0;
	currentCommand = 0;
	currentPoint = 0;
	printingCompletation = 0;

}

bool ofxDobot::updateSVG(){
    
    
    bool isPrinting = true;

    if ( currentPath < polylines.size() ){
        
        if ( getQueuedCmdLeftSpace() > 0 ){
            if ( currentCommand < polylines[currentPath]->size()){


                CPCmd cmd;
                cmd.cpMode = 1;
        
                ofPoint to = convertToDobotCoordinate(polylines[currentPath]->getVertices()[currentCommand]);
                cmd.x = to.x;
                cmd.y = to.y;
                cmd.z = autoZ;
                //cout << "moveTo" << endl;
                setCPCmd(cmd);
                currentCommand++;
                currentPoint++;

				int printPercentage = printingCompletation;
				printingCompletation = (float)currentPoint / totalNumPoints * 100;
				if (printPercentage != (int)printingCompletation) {
					ofLog(OF_LOG_NOTICE, name + " " + ofToString(printPercentage) + "% drawing");
				}
            }
            
            else{
                
                currentPath++;
                currentCommand = 0;
                
            }
        }
       
    }
    else{
        if ( getQueuedCmdLeftSpace() >= DOBOT_QUEUE_LENGTH ){
            ofLog(OF_LOG_NOTICE, "svg finished");
			int lastVertices = polylines[polylines.size() - 1]->getVertices().size() - 1;
			CPCmd cmd;
			cmd.cpMode = 1;
			ofPoint to = convertToDobotCoordinate(polylines[polylines.size()-1]->getVertices()[lastVertices]);
			cmd.x = to.x;
			cmd.y = to.y;
			cmd.z = 45;

			setCPCmd(cmd);
            isPrinting = false;
            typeFileOpened = NONE;
            
        }
        
    }
    return isPrinting;
}

bool ofxDobot::updateXML(){
    
    bool isRunning = true;
    if (timeline.tagExists("row" + ofToString(rowIndex))){
        
         if ( getQueuedCmdLeftSpace() >= 3 ){
        
            if (timeline.tagExists("vel" + ofToString(rowIndex))) {
                timeline.pushTag("vel" + ofToString(rowIndex));
                float vel = timeline.getValue("vel", 0);
                float acc = timeline.getValue("acc", 0);
                setPTPCommonParams(true, vel, acc);
                timeline.popTag();
                ofLog(OF_LOG_VERBOSE, "vel " + ofToString(rowIndex));
            }
            
            
            timeline.pushTag("row" + ofToString(rowIndex));
            ofLog(OF_LOG_VERBOSE, "row " + ofToString(rowIndex));
            int type = timeline.getValue("item_0", 0);
            double x = timeline.getValue("item_2", 0);
            double y = timeline.getValue("item_3", 0);
            double z = timeline.getValue("item_4", 0);
            double r = timeline.getValue("item_5", 0);
            double timePause = timeline.getValue("item_10", 0);
            
            
            setPTPCmd((ptpMode)type, x, y, z, r);
            
            
            
            if (timePause > 0) {
                WAITCmd waitCmd;
                waitCmd.timeout = timePause * 1000;
                
                setWAITCmd(waitCmd);
            }
            
			ofLog(OF_LOG_NOTICE, ofToString(rowIndex) + "num row");
            rowIndex++;
            timeline.popTag();
         }
    }
    else if ( getQueuedCmdLeftSpace() >=  DOBOT_QUEUE_LENGTH ){
        isRunning = false;
        typeFileOpened = NONE;
    }
   
    return isRunning;
}

bool ofxDobot::updateText(){
    
    bool isRunning = true;
    if ( rowIndex < lines.size() ) {
        
        if ( getQueuedCmdLeftSpace() > 0 ){
            vector<string> splittedLine = ofSplitString(lines[rowIndex], " ");
            
            if (splittedLine[0] == "PTPCommonParams") {
                if (splittedLine.size() >= 4)
                    setPTPCommonParams(ofToBool(splittedLine[1]), ofToFloat(splittedLine[2]), ofToFloat(splittedLine[3]));
                else
                    ofLog(OF_LOG_ERROR, "PTPCommonParams error line " + ofToString(rowIndex));
                
            }
            else if (splittedLine[0] == "PTPCmd") {
                if (splittedLine.size() >= 6)
                    setPTPCmd((ptpMode)ofToInt(splittedLine[1]), ofToFloat(splittedLine[2]), ofToFloat(splittedLine[3]), ofToFloat(splittedLine[4]), ofToFloat(splittedLine[5]));
                else
                    ofLog(OF_LOG_ERROR, "PTPCmd error line " + ofToString(rowIndex));
            }
            else if (splittedLine[0] == "WAITCmd") {
                if (splittedLine.size() >= 2) {
                    WAITCmd cmd;
                    cmd.timeout = ofToInt(splittedLine[1]);
                    setWAITCmd(cmd);
                }
                else {
                    ofLog(OF_LOG_ERROR, "WAITCmd error line " + ofToString(rowIndex));
                }
                
                
            }
            rowIndex++;
            ofLog(OF_LOG_VERBOSE, "line  " + ofToString(rowIndex));
        }
    }
    else{
        ofLog(OF_LOG_NOTICE,"text file finished");
        if ( getQueuedCmdLeftSpace() > DOBOT_QUEUE_LENGTH ){
            isRunning = false;
            typeFileOpened = NONE;
        }
    }
    return isRunning;

}



bool ofxDobot::update(){

    bool isRunning = false;
	if (!isStopped) {
		if (typeFileOpened == XML) {
			isRunning = updateXML();
		}
		else if (typeFileOpened == TXT) {
			isRunning = updateText();
		}
		else if (typeFileOpened == SVG) {
			isRunning = updateSVG();
		}
	}

    return isRunning;
    
}





string ofxDobot::getDeviceSN() {


	if (connected) {
		uint8_t  message[7];
		message[0] = 0xAA;
		message[1] = 0xAA;
		message[2] = 2;
		message[3] = DeviceSN;
		message[4] = 0;
		message[4] |= 0 & 0x01;
		message[4] |= (0 << 1) & 0x02;
		message[5] = 0 - (message[3] + message[4]);
		int result = serial.writeBytes(message, 7);
		if (result == OF_SERIAL_ERROR) {
			ofLog(OF_LOG_ERROR, "error on get device SN");
			return "";
		}
		else {

            timeMessage = ofGetElapsedTimeMillis();
			waitingMessage = true;
			while (waitingMessage) {
				yield();
			}
			ofLog(OF_LOG_VERBOSE,"serialNumber = "+serialNumber);
			return serialNumber;
		}

	}

	else {
		ofLog(OF_LOG_ERROR, noConnection);
		return "";
	}

}

string ofxDobot::getName(){

	if (connected) {
		uint8_t  message[6];
		message[0] = 0xAA;
		message[1] = 0xAA;
		message[2] = 2;
		message[3] = DeviceName;
		message[4] = 0;
		message[4] |= 0 & 0x01;
		message[4] |= (0 << 1) & 0x02;
		message[5] = 0 - (message[3] + message[4]);
		int result = serial.writeBytes(message, 6);
		if (result == OF_SERIAL_ERROR) {
			ofLog(OF_LOG_ERROR, "error on get device Name");
			return "";
		}
		else {
            timeMessage = ofGetElapsedTimeMillis();
			waitingMessage = true;
			while (waitingMessage) {
				yield();
			}
			ofLog(OF_LOG_VERBOSE,"name = "+name);
			return name;

		}
	}
	else {
		ofLog(OF_LOG_ERROR, noConnection);
		return "";
	}



}

Pose ofxDobot::getPose() {

	if (connected && !waitingMessage) {
		uint8_t  message[6];
		message[0] = 0xAA;
		message[1] = 0xAA;
		message[2] = 2;
		message[3] = 10;
		message[4] = 0;
		message[4] |= 0 & 0x01;
		message[4] |= (0 << 1) & 0x02;

		uint8_t checksum = 0;
		for (int i = 0; i < message[2]; i++) {
			checksum += message[i + 3];
		}
		message[5] = 0 - checksum;
		int result = serial.writeBytes(message, 6);
		if (result == OF_SERIAL_ERROR) {
			ofLog(OF_LOG_ERROR, "get Pose serial error");

		}
		else {
			timeMessage = ofGetElapsedTimeMillis();
			waitingMessage = true;
			while (waitingMessage) {
				yield();
			}


		}

		return pose;
	}

	else {
		if (!connected) {
			ofLog(OF_LOG_ERROR, noConnection);
		}
	}
    return pose;
}


void ofxDobot::enableUpdatePose(bool enable){
    
    
    automaticUpdatePose = enable;
    
}

void ofxDobot::updatePose(){
    
    if (connected && !waitingMessage ) {
        uint8_t  message[6];
        message[0] = 0xAA;
        message[1] = 0xAA;
        message[2] = 2;
        message[3] = 10;
        message[4] = 0;
        message[4] |= 0 & 0x01;
        message[4] |= (0 << 1) & 0x02;
        
        uint8_t checksum = 0;
        for (int i = 0; i < message[2]; i++) {
            checksum += message[i + 3];
        }
        message[5] = 0 - checksum;
        int result = serial.writeBytes(message, 6);
        if (result == OF_SERIAL_ERROR) {
            ofLog(OF_LOG_ERROR, "get Pose serial error");
            
        }
        else {
            timeMessage = ofGetElapsedTimeMillis();
            waitingMessage = true;
           
            //return pose;
            
        }
    }
    
    else {
        if ( !connected ){
            ofLog(OF_LOG_ERROR, noConnection);
        }
    }
    
    
}



void ofxDobot::resetPose(uint8_t manual, float rearArmAngle, float frontArmAngle) {


	if (connected) {
		uint8_t  message[15];
		message[0] = 0xAA;
		message[1] = 0xAA;
		message[2] = 11;
		message[3] = ResetPose;
		message[4] = 0;
		message[4] |= 1 & 0x01;
		message[4] |= (0 << 1) & 0x02;
		message[5] = manual;


		memcpy(&message[6], &rearArmAngle,4);
		memcpy(&message[10], &frontArmAngle, 4);

		uint8_t checksum = 0;
		for (int i = 0; i < message[2]; i++) {
			checksum += message[i + 3];
		}
		message[14] = 0 - checksum;
		int result = serial.writeBytes(message, 15);
		if (result == OF_SERIAL_ERROR) {
			ofLog(OF_LOG_ERROR, "reset pose error");
		}

	}

	else {
		ofLog(OF_LOG_ERROR, noConnection);
	}



}

void ofxDobot::setAutoZ(){
    
    autoZ = getPose().z;
    cout << autoZ << endl;
}


void ofxDobot::getAlarmsState(uint8_t alarmsState[16]) {

	if (connected) {
		uint8_t  message[6];
		message[0] = 0xAA;
		message[1] = 0xAA;
		message[2] = 2;
		message[3] = GetAlarms;
		message[4] = 0;
		message[4] |= 0 & 0x01;
		message[4] |= (0 << 1) & 0x02;
		message[5] = 0 - (message[3] + message[4]);
		int result = serial.writeBytes(message, 6);
		if (result == OF_SERIAL_ERROR) {
			ofLog(OF_LOG_ERROR, "error on get alarms state");
		}
		else {
			this->alarmsState = alarmsState;
			waitingMessage = true;
			while (waitingMessage) {
				yield();
			}

		}

	}

	else {
		ofLog(OF_LOG_ERROR, noConnection);
	}



}

void ofxDobot::clearAllAlarmsState() {

	if (connected) {
		uint8_t  message[6];
		message[0] = 0xAA;
		message[1] = 0xAA;
		message[2] = 2;
		message[3] = ClearAllAlarmsState;
		message[4] = 0;
		message[4] |= 1 & 0x01;
		message[4] |= (0 << 1) & 0x02;
		message[5] = 0 - (message[3] + message[4]);
		int result = serial.writeBytes(message, 6);
		if (result == OF_SERIAL_ERROR) {
			ofLog(OF_LOG_ERROR, "error on clear alarms state");
		}
	}

	else {
		ofLog(OF_LOG_ERROR, noConnection);
	}



}


void ofxDobot::setPTPJointParams(bool isQueued, float velocity[4], float acceleration[4]) {

	if (connected) {
		uint8_t message[38];
		message[0] = 0xAA;
		message[1] = 0xAA;
		message[2] = 34;
		message[3] = PTPJointParams;
		message[4] = 0;
		message[4] |= 1 & 0x01;
		message[4] |= (isQueued << 1) & 0x02;

		memcpy(&message[5], velocity, 16);
		memcpy(&message[21], acceleration, 16);


		uint8_t checksum = 0;
		for (int i = 0; i < message[2]; i++) {
			checksum += message[i + 3];
		}
		message[37] = 0 - checksum;
		int result = serial.writeBytes(message, 38);
		if (result == OF_SERIAL_ERROR) {
			ofLog(OF_LOG_ERROR, "serial error PTPJointParams");
		}
	}

	else {
		ofLog(OF_LOG_ERROR, noConnection);
	}
}

void ofxDobot::setPTPCoordinateParams(bool isQueued, float xyzVelocity, float rVelocity, float xyzAcceleration, float rAcceleration){

	if (connected) {
		uint8_t message[22];
		message[0] = 0xAA;
		message[1] = 0xAA;
		message[2] = 18;
		message[3] = PTPCoordinateParams;
		message[4] = 0;
		message[4] |= 1 & 0x01;
		message[4] |= (isQueued << 1) & 0x02;

		memcpy(&message[5], &xyzVelocity, 4);
		memcpy(&message[9], &rVelocity, 4);
		memcpy(&message[13], &xyzAcceleration, 4);
		memcpy(&message[17], &rAcceleration, 4);


		uint8_t checksum = 0;
		for (int i = 0; i < message[2]; i++) {
			checksum += message[i + 3];
		}
		message[21] = 0 - checksum;
		int result = serial.writeBytes(message, 22);
		if (result == OF_SERIAL_ERROR) {
			ofLog(OF_LOG_ERROR, "serial error setPTPCoordinateParams");
		}

	}

	else {
		ofLog(OF_LOG_ERROR, noConnection);
	}
}

void ofxDobot::setPTPJumpParams(bool isQueued, float jumpHeight, float zLimit) {

	if (connected) {
		uint8_t message[14];
		message[0] = 0xAA;
		message[1] = 0xAA;
		message[2] = 10;
		message[3] = PTPJumpParams;
		message[4] = 0;
		message[4] |= 1 & 0x01;
		message[4] |= (isQueued << 1) & 0x02;

		memcpy(&message[5], &jumpHeight, 4);
		memcpy(&message[9], &zLimit, 4);


		uint8_t checksum = 0;
		for (int i = 0; i < message[2]; i++) {
			checksum += message[i + 3];
		}
		message[13] = 0 - checksum;
		int result = serial.writeBytes(message, 14);
		if (result == OF_SERIAL_ERROR) {
			ofLog(OF_LOG_ERROR, "serial error PTPJumpParams");
		}

	}

	else {
		ofLog(OF_LOG_ERROR, noConnection);
	}
}

void ofxDobot::setPTPCommonParams(bool isQueued, float velocityRation, float accelerationRatio) {

	if (connected) {
		uint8_t message[14];
		message[0] = 0xAA;
		message[1] = 0xAA;
		message[2] = 10;
		message[3] = PTPCommonParams;

		message[4] = 0;
		message[4] |= 1 & 0x01;
		message[4] |= (isQueued << 1) & 0x02;


		memcpy(&message[5], &velocityRation, 4);
		memcpy(&message[9], &accelerationRatio, 4);


		uint8_t checksum = 0;
		for (int i = 0; i < message[2]; i++) {
			checksum += message[i + 3];
		}
		message[13] = 0 - checksum;
		int result = serial.writeBytes(message, 14);
		if (result == OF_SERIAL_ERROR) {
			ofLog(OF_LOG_ERROR, "serial error PTPCommonParams");
		}

	}

	else {
		ofLog(OF_LOG_ERROR, noConnection);
	}

}

void ofxDobot::setPTPCmd(ptpMode mode, float x, float y, float z, float r) {

	if (connected) {
		uint8_t message[23];
		message[0] = 0xAA;
		message[1] = 0xAA;
		message[2] = 19;
		message[3] = PTPCmd;
		message[4] = 0;
		message[4] |= 1 & 0x01;
		message[4] |= (1 << 1) & 0x02;
		message[5] = mode;

		memcpy(&message[6], &x, 4);
		memcpy(&message[10], &y, 4);
		memcpy(&message[14], &z, 4);
		memcpy(&message[18], &r, 4);


		uint8_t checksum = 0;
		for (int i = 0; i < message[2]; i++) {
			checksum += message[i + 3];
		}
		message[22] = 0 - checksum;
		int result = serial.writeBytes(message, 23);
		if (result == OF_SERIAL_ERROR) {
			ofLog(OF_LOG_ERROR, "serial error PTPCmd");
		}

	}

	else {
		ofLog(OF_LOG_ERROR,noConnection);
	}

}

void ofxDobot::setJOGJointParams(bool isQueue, JOGJointParams params) {

	if (connected) {
		uint8_t  message[38];
		message[0] = 0xAA;
		message[1] = 0xAA;
		message[2] = 34;
		message[3] = SETGETJOGJointParams;
		message[4] = 0;
		message[4] |= 1 & 0x01;
		message[4] |= (isQueue << 1) & 0x02;
		memcpy(&message[5], params.velocity, 16);
		memcpy(&message[21], params.acceleration, 16);

		uint8_t checksum = 0;
		for (int i = 0; i < message[2]; i++) {
			checksum += message[i + 3];
		}
		message[37] = 0 - checksum;
		int result = serial.writeBytes(message, 38);
		if (result == OF_SERIAL_ERROR) {
			ofLog(OF_LOG_ERROR, "serial error setJOGJointParams");
		}

	}

	else {
		ofLog(OF_LOG_ERROR, noConnection);
	}




}

JOGJointParams ofxDobot::getJOGJointParams() {

	if (connected) {
		uint8_t  message[6];
		message[0] = 0xAA;
		message[1] = 0xAA;
		message[2] = 2;
		message[3] = SETGETJOGJointParams;
		message[4] = 0;
		message[4] |= 0 & 0x01;
		message[4] |= (0 << 1) & 0x02;

		uint8_t checksum = 0;
		for (int i = 0; i < message[2]; i++) {
			checksum += message[i + 3];
		}
		message[5] = 0 - checksum;
		int result = serial.writeBytes(message, 6);
		if (result == OF_SERIAL_ERROR) {
			ofLog(OF_LOG_ERROR, "serial error getJOGJointParams");
		}
		else {
			waitingMessage = true;
			while (waitingMessage) {
				yield();
			}
		}
	}

	else {
		ofLog(OF_LOG_ERROR, noConnection);
	}
}

void ofxDobot::setJOGCoordinateParams(bool isQueue, JOGCoordinateParams params) {

	if (connected) {
		uint8_t  message[38];
		message[0] = 0xAA;
		message[1] = 0xAA;
		message[2] = 34;
		message[3] = SETGETJOGCoordinateParams;
		message[4] = 0;
		message[4] |= 1 & 0x01;
		message[4] |= (isQueue << 1) & 0x02;
		memcpy(&message[5], params.velocity, 16);
		memcpy(&message[21], params.acceleration, 16);

		uint8_t checksum = 0;
		for (int i = 0; i < message[2]; i++) {
			checksum += message[i + 3];
		}
		message[37] = 0 - checksum;
		int result = serial.writeBytes(message, 38);
		if (result == OF_SERIAL_ERROR) {
			ofLog(OF_LOG_ERROR, "serial error setJOGJointParams");
		}
	}
	else {
		ofLog(OF_LOG_ERROR, noConnection);
	}
}

JOGCoordinateParams ofxDobot::getJOGCoordinateParams() {

	if (connected) {
		uint8_t  message[6];
		message[0] = 0xAA;
		message[1] = 0xAA;
		message[2] = 2;
		message[3] = SETGETJOGCoordinateParams;
		message[4] = 0;
		message[4] |= 0 & 0x01;
		message[4] |= (0 << 1) & 0x02;

		uint8_t checksum = 0;
		for (int i = 0; i < message[2]; i++) {
			checksum += message[i + 3];
		}
		message[5] = 0 - checksum;
		int result = serial.writeBytes(message, 6);
		if (result == OF_SERIAL_ERROR) {
			ofLog(OF_LOG_ERROR, "serial error getJOGJointParams");
		}
		else {
			waitingMessage = true;
			while (waitingMessage) {
				yield();
			}
		}
	}

	else {
		ofLog(OF_LOG_ERROR, noConnection);
	}
	return jogCoordinateParams;
}


void ofxDobot::setJOGCommonParams(bool isQueue, JOGCommonParams params) {

	if (connected) {
		uint8_t  message[14];
		message[0] = 0xAA;
		message[1] = 0xAA;
		message[2] = 10;
		message[3] = SETGETJOGCommonParams;
		message[4] = 0;
		message[4] |= 1 & 0x01;
		message[4] |= (isQueue << 1) & 0x02;
		memcpy(&message[5], &params.velocityRatio, 4);
		memcpy(&message[9], &params.accelerationRatio, 4);

		uint8_t checksum = 0;
		for (int i = 0; i < message[2]; i++) {
			checksum += message[i + 3];
		}
		message[13] = 0 - checksum;
		int result = serial.writeBytes(message, 14);
		if (result == OF_SERIAL_ERROR) {
			ofLog(OF_LOG_ERROR, "serial error setJOGJointParams");
		}
	}
	else {
		ofLog(OF_LOG_ERROR, noConnection);
	}
}

JOGCommonParams ofxDobot::getJOGCommonParams() {

	if (connected) {
		uint8_t  message[6];
		message[0] = 0xAA;
		message[1] = 0xAA;
		message[2] = 2;
		message[3] = SETGETJOGCommonParams;
		message[4] = 0;
		message[4] |= 0 & 0x01;
		message[4] |= (0 << 1) & 0x02;

		uint8_t checksum = 0;
		for (int i = 0; i < message[2]; i++) {
			checksum += message[i + 3];
		}
		message[5] = 0 - checksum;
		int result = serial.writeBytes(message, 6);
		if (result == OF_SERIAL_ERROR) {
			ofLog(OF_LOG_ERROR, "serial error getJOGCommonParams");
		}
		else {
			waitingMessage = true;
			while (waitingMessage) {
				yield();
			}
		}
	}

	else {
		ofLog(OF_LOG_ERROR, noConnection);
	}
	return jogCommonParams;
}

void ofxDobot::setJOGCmd(JOGCmd cmd) {

		if (connected) {
			uint8_t  message[8];
			message[0] = 0xAA;
			message[1] = 0xAA;
			message[2] = 4;
			message[3] = SetJOGCmd;
			message[4] = 0;
			message[4] |= 1 & 0x01;
			message[4] |= (1 << 1) & 0x02;
			message[5] = cmd.isJoint;
			message[6] = cmd.cmd;

			uint8_t checksum = 0;
			for (int i = 0; i < message[2]; i++) {
				checksum += message[i + 3];
			}
			message[7] = 0 - checksum;
			int result = serial.writeBytes(message, 8);
			if (result == OF_SERIAL_ERROR) {
				ofLog(OF_LOG_ERROR, "serial error JOGCmd");
			}

		}

		else {
			ofLog(OF_LOG_ERROR, noConnection);
		}
}

void ofxDobot::setCPParams(bool isQueue, CPParams params) {

	if (connected) {
		uint8_t  message[19];
		message[0] = 0xAA;
		message[1] = 0xAA;
		message[2] = 15;
		message[3] = SETGETCPParams;
		message[4] = 0;
		message[4] |= 1 & 0x01;
		message[4] |= (isQueue << 1) & 0x02;

		memcpy(&message[5], &params.planAcc, 4);
		memcpy(&message[9], &params.junctionVel, 4);

		params.realTimeTrack = message[17];

		if ( params.realTimeTrack )
			memcpy(&message[13], &params.acc, 4);
		else {
			memcpy(&message[13], &params.period, 4);
		}

		uint8_t checksum = 0;
		for (int i = 0; i < message[2]; i++) {
			checksum += message[i + 3];
		}
		message[18] = 0 - checksum;
		int result = serial.writeBytes(message, 19);
		if (result == OF_SERIAL_ERROR) {
			ofLog(OF_LOG_ERROR, "serial error setCPParams");
		}
	}

	else {
		ofLog(OF_LOG_ERROR, noConnection);
	}


}

CPParams ofxDobot::getCPParams() {

	if (connected) {
		uint8_t  message[6];
		message[0] = 0xAA;
		message[1] = 0xAA;
		message[2] = 2;
		message[3] = SETGETCPParams;
		message[4] = 0;
		message[4] |= 0 & 0x01;
		message[4] |= (0 << 1) & 0x02;

		uint8_t checksum = 0;
		for (int i = 0; i < message[2]; i++) {
			checksum += message[i + 3];
		}
		message[5] = 0 - checksum;
		int result = serial.writeBytes(message, 6);
		if (result == OF_SERIAL_ERROR) {
			ofLog(OF_LOG_ERROR, "serial error getCPParams");
		}
		else {
			waitingMessage = true;
			while (waitingMessage) {
				yield();
			}
		}
	}

	else {
		ofLog(OF_LOG_ERROR, noConnection);
	}

	return cpParams;

}

void ofxDobot::setCPCmd(CPCmd cmd) {

	if (connected) {
		uint8_t  message[23];
		message[0] = 0xAA;
		message[1] = 0xAA;
		message[2] = 19;
		message[3] = SetCPCmd;
		message[4] = 0;
		message[4] |= 1 & 0x01;
		message[4] |= (1 << 1) & 0x02;

		message[5] = cmd.cpMode;

		memcpy(&message[6], &cmd.x, 4);
		memcpy(&message[10], &cmd.y, 4);
		memcpy(&message[14], &cmd.z, 4);
		memcpy(&message[18], &cmd.power, 4);

		uint8_t checksum = 0;
		for (int i = 0; i < message[2]; i++) {
			checksum += message[i + 3];
		}
		message[22] = 0 - checksum;
		int result = serial.writeBytes(message, 23);
		if (result == OF_SERIAL_ERROR) {
			ofLog(OF_LOG_ERROR, "serial error setCPCmd");
		}
	}

	else {
		ofLog(OF_LOG_ERROR, noConnection);
	}


}

void ofxDobot::setARCParams(bool isQueue, ARCParams params){

    if (connected) {
        uint8_t  message[22];
        message[0] = 0xAA;
        message[1] = 0xAA;
        message[2] = 18;
        message[3] = SetGetARCParams;
        message[4] = 0;
        message[4] |= 1 & 0x01;
        message[4] |= (isQueue << 1) & 0x02;
        
        memcpy(&message[5], &params.xyzVelocity, 4);
        memcpy(&message[9], &params.rVelocity, 4);
        memcpy(&message[13], &params.xyzAcceleration, 4);
        memcpy(&message[17], &params.rAcceleration, 4);
        
       
        
        uint8_t checksum = 0;
        for (int i = 0; i < message[2]; i++) {
            checksum += message[i + 3];
        }
        message[21] = 0 - checksum;
        int result = serial.writeBytes(message, 22);
        if (result == OF_SERIAL_ERROR) {
            ofLog(OF_LOG_ERROR, "serial error setARCParams");
        }
    }
    
    else {
        ofLog(OF_LOG_ERROR, noConnection);
    }

    
}

void ofxDobot::setARCCmd(ARCCmd cmd){
    
    if (connected) {
        uint8_t  message[38];
        message[0] = 0xAA;
        message[1] = 0xAA;
        message[2] = 34;
        message[3] = SetGetARCCmd;
        message[4] = 0;
        message[4] |= 1 & 0x01;
        message[4] |= (1 << 1) & 0x02;
        

        memcpy(&message[5], &cmd.cirPoint.x, 4);
        memcpy(&message[9], &cmd.cirPoint.y, 4);
        memcpy(&message[13], &cmd.cirPoint.z, 4);
        memcpy(&message[17], &cmd.cirPoint.r, 4);
        memcpy(&message[21], &cmd.toPoint.x, 4);
        memcpy(&message[25], &cmd.toPoint.y, 4);
        memcpy(&message[29], &cmd.toPoint.z, 4);
        memcpy(&message[33], &cmd.toPoint.r, 4);
        
        uint8_t checksum = 0;
        for (int i = 0; i < message[2]; i++) {
            checksum += message[i + 3];
        }
        message[37] = 0 - checksum;
        int result = serial.writeBytes(message, 38);
        if (result == OF_SERIAL_ERROR) {
            ofLog(OF_LOG_ERROR, "serial error setARCCmd");
        }
    }
    
    else {
        ofLog(OF_LOG_ERROR, noConnection);
    }

    
}

void ofxDobot::setHomeParams(bool isQueue, HOMEParams params) {

	if (connected) {
		uint8_t  message[22];
		message[0] = 0xAA;
		message[1] = 0xAA;
		message[2] = 18;
		message[3] = HomeParams;
		message[4] = 0;
		message[4] |= 1 & 0x01;
		message[4] |= (isQueue << 1) & 0x02;

		memcpy(&message[5], &params.x, 4);
		memcpy(&message[9], &params.y, 4);
		memcpy(&message[13], &params.z, 4);
		memcpy(&message[17], &params.r, 4);

		uint8_t checksum = 0;
		for (int i = 0; i < message[2]; i++) {
			checksum += message[i + 3];
		}
		message[21] = 0 - checksum;
		int result = serial.writeBytes(message, 22);
		if (result == OF_SERIAL_ERROR) {
			ofLog(OF_LOG_ERROR, "serial error sethomeParams");
		}
	}

	else {
		ofLog(OF_LOG_ERROR, noConnection);
	}
}

void ofxDobot::setHomeCmd() {

	if (connected) {
		uint8_t  message[7];
		message[0] = 0xAA;
		message[1] = 0xAA;
		message[2] = 3;
		message[3] = HomeCmd;
		message[4] = 0;
		message[4] |= 1 & 0x01;
		message[4] |= (1 << 1) & 0x02;
		message[5] = 0;

		uint8_t checksum = 0;
		for (int i = 0; i < message[2]; i++) {
			checksum += message[i + 3];
		}
		message[6] = 0 - checksum;
		int result = serial.writeBytes(message, 7);
		if (result == OF_SERIAL_ERROR) {
			ofLog(OF_LOG_ERROR, "serial error sethomeCmd");
		}
	}

	else {
		ofLog(OF_LOG_ERROR, noConnection);
	}
}

void ofxDobot::setWAITCmd(WAITCmd cmd) {

	if (connected) {
		uint8_t  message[10];
		message[0] = 0xAA;
		message[1] = 0xAA;
		message[2] = 6;
		message[3] = SetWAITCmd;
		message[4] = 0;
		message[4] |= 1 & 0x01;
		message[4] |= (1 << 1) & 0x02;

		memcpy(&message[5], &cmd.timeout, 4);


		uint8_t checksum = 0;
		for (int i = 0; i < message[2]; i++) {
			checksum += message[i + 3];
		}
		message[9] = 0 - checksum;
		int result = serial.writeBytes(message, 10);
		if (result == OF_SERIAL_ERROR) {
			ofLog(OF_LOG_ERROR, "serial error setWAITCmd");
		}
	}

	else {
		ofLog(OF_LOG_ERROR, noConnection);
	}


}

void ofxDobot::setAngleSensorStaticError(ArmAngleError armAngle) {

	if (connected) {
		uint8_t  message[14];
		message[0] = 0xAA;
		message[1] = 0xAA;
		message[2] = 10;
		message[3] = AngleSensorStaticError;
		message[4] = 0;
		message[4] |= 1 & 0x01;
		message[4] |= (0 << 1) & 0x02;

		memcpy(&message[5], &armAngle.rearArmAngleError, 4);
		memcpy(&message[9], &armAngle.frontArmAngleError, 4);


		uint8_t checksum = 0;
		for (int i = 0; i < message[2]; i++) {
			checksum += message[i + 3];
		}
		message[13] = 0 - checksum;
		int result = serial.writeBytes(message, 14);
		if (result == OF_SERIAL_ERROR) {
			ofLog(OF_LOG_ERROR, "serial error setAngleSensorStaticError");
		}
	}

	else {
		ofLog(OF_LOG_ERROR, noConnection);
	}


}

ArmAngleError ofxDobot::getAngleSensorStaticError() {

	if (connected) {
		uint8_t  message[6];
		message[0] = 0xAA;
		message[1] = 0xAA;
		message[2] = 2;
		message[3] = AngleSensorStaticError;
		message[4] = 0;
		message[4] |= 0 & 0x01;
		message[4] |= (0 << 1) & 0x02;

		uint8_t checksum = 0;
		for (int i = 0; i < message[2]; i++) {
			checksum += message[i + 3];
		}
		message[5] = 0 - checksum;
		int result = serial.writeBytes(message, 6);
		if (result == OF_SERIAL_ERROR) {
			ofLog(OF_LOG_ERROR, "serial error getAngleSensorStaticError");
		}
		else {
			waitingMessage = true;
			while (waitingMessage) {
				yield();
			}
		}
	}

	else {
		ofLog(OF_LOG_ERROR, noConnection);
	}

	return armAngleError;

}


void ofxDobot::setQueuedCmdStartExec() {

	if (connected) {
		uint8_t  message[6];
		message[0] = 0xAA;
		message[1] = 0xAA;
		message[2] = 2;
		message[3] = SetQueuedCmdStartExec;
		message[4] = 0;
		message[4] |= 1 & 0x01;
		message[4] |= (0 << 1) & 0x02;

		uint8_t checksum = 0;
		for (int i = 0; i < message[2]; i++) {
			checksum += message[i + 3];
		}
		message[5] = 0 - checksum;
		int result = serial.writeBytes(message, 7);
		if (result == OF_SERIAL_ERROR) {
			ofLog(OF_LOG_ERROR, "serial error setQueueCmdStartExec");
		}
		else
			timeMessage = ofGetElapsedTimeMillis();

	}

	else {
		ofLog(OF_LOG_ERROR,noConnection);
	}


}

void ofxDobot::setQueuedCmdStopExec() {

	if (connected) {
		uint8_t  message[6];
		message[0] = 0xAA;
		message[1] = 0xAA;
		message[2] = 2;
		message[3] = SetQueuedCmdStopExec;
		message[4] = 0;
		message[4] |= 1 & 0x01;
		message[4] |= (0 << 1) & 0x02;

		uint8_t checksum = 0;
		for (int i = 0; i < message[2]; i++) {
			checksum += message[i + 3];
		}
		message[5] = 0 - checksum;
		int result = serial.writeBytes(message, 7);
		if (result == OF_SERIAL_ERROR) {
			ofLog(OF_LOG_ERROR, "serial error setQueueCmdStopExec");
		}
	}
	else {
		ofLog(OF_LOG_ERROR, noConnection);
	}
}

void ofxDobot::setQueuedCmdClear() {

	if (connected) {
		uint8_t  message[6];
		message[0] = 0xAA;
		message[1] = 0xAA;
		message[2] = 2;
		message[3] = SetQueuedCmdClear;
		message[4] = 0;
		message[4] |= 1 & 0x01;
		message[4] |= (0 << 1) & 0x02;

		uint8_t checksum = 0;
		for (int i = 0; i < message[2]; i++) {
			checksum += message[i + 3];
		}
		message[5] = 0 - checksum;
		int result = serial.writeBytes(message, 7);
		if (result == OF_SERIAL_ERROR) {
			ofLog(OF_LOG_ERROR, "serial error setQueueCmdClear");
		}
		else {
			timeMessage = ofGetElapsedTimeMillis();
			waitingMessage = false;
		}

	}

	else {
		ofLog(OF_LOG_ERROR, noConnection);
	}
}

int ofxDobot::getQueuedCmdCurrentIndex() {

	if (connected) {
		uint8_t  message[6];
		message[0] = 0xAA;
		message[1] = 0xAA;
		message[2] = 2;
		message[3] = GetQueuedCmdCurrentIndex;
		message[4] = 0;
		message[4] |= 0 & 0x01;
		message[4] |= (0 << 1) & 0x02;

		uint8_t checksum = 0;
		for (int i = 0; i < message[2]; i++) {
			checksum += message[i + 3];
		}
		message[5] = 0 - checksum;
		int result = serial.writeBytes(message, 7);
		if (result == OF_SERIAL_ERROR) {
			ofLog(OF_LOG_ERROR, "serial error get queued cmd index");

		}
		else {
			timeMessage = ofGetElapsedTimeMillis();
			waitingMessage = true;
			while (waitingMessage) {
				yield();
			}
		}

	}

	else {
		ofLog(OF_LOG_ERROR, noConnection);
	}

	return queuedCmdIndex;

}

int ofxDobot::getQueuedCmdLeftSpace() {

	if (connected) {
		uint8_t  message[6];
		message[0] = 0xAA;
		message[1] = 0xAA;
		message[2] = 2;
		message[3] = GetQueuedCmdLeftSpace;
		message[4] = 0;
		message[4] |= 0 & 0x01;
		message[4] |= (0 << 1) & 0x02;

		uint8_t checksum = 0;
		for (int i = 0; i < message[2]; i++) {
			checksum += message[i + 3];
		}
		message[5] = 0 - checksum;
		int result = serial.writeBytes(message, 6);
		if (result == OF_SERIAL_ERROR) {
			ofLog(OF_LOG_ERROR, "serial error get queued left space");
			queuedLeftSpace  = 0;

		}
		else {
            timeMessage = ofGetElapsedTimeMillis();
			waitingMessage = false;
			while (waitingMessage) {
				yield();
			}
		}

	}

	else {
		ofLog(OF_LOG_ERROR, noConnection);
	}

	return queuedLeftSpace;

}

void ofxDobot::elaborateParams(int idProtocol, vector<uint8_t> params){
    
    switch(idProtocol){
        case DeviceSN:
            serialNumber.clear();
            for (int i = 0; i < params.size(); i++) {
                const char c = params[i];
                serialNumber.append(&c, 1);
            }
			waitingMessage = false;
			break;
            
        case GetQueuedCmdLeftSpace:
            //ofLog(OF_LOG_VERBOSE, "GetQueuedCmdLeftSpace");
            memcpy(&queuedLeftSpace, &params[0], 4);
            ofLog(OF_LOG_VERBOSE, "queued left space = " + ofToString(queuedLeftSpace));
			waitingMessage = false;
			break;
            
        case DeviceName:
            name.clear();
            for (int i = 0; i < params.size(); i++) {
                const char c = params[i];
                name.append(&c, 1);
            }
			waitingMessage = false;
			break;
            
        case GetPose:
            
            memcpy(&pose.x, &params[0], 4);
            memcpy(&pose.y, &params[4], 4);
            memcpy(&pose.z, &params[8], 4);
            memcpy(&pose.r, &params[12], 4);
            memcpy(pose.jointAngle, &params[16], 16);
			waitingMessage = false;
			break;
            
		case SETGETCPParams:
			memcpy(&cpParams.acc, &params[0], 4);
			memcpy(&cpParams.junctionVel, &params[4], 4);
			memcpy(&cpParams.realTimeTrack, &params[12], 1);
			if (cpParams.realTimeTrack) {
				memcpy(&cpParams.period, &params[8], 4);
			}
			else {
				memcpy(&cpParams.planAcc, &params[8], 4);
			}
			waitingMessage = false;
			break;
        default:
			ofLog(OF_LOG_VERBOSE, ofToString(idProtocol));
            waitingMessage = false;
            break;
    }
}

void ofxDobot::threadedFunction() {


	while (isThreadRunning()) {

        sleep(16);
		
		if (connected) {
            
			//update();

            if ( waitingMessage &&  ofGetElapsedTimeMillis() - timeMessage > cmdTimeout ){
                
                ofLog(OF_LOG_ERROR,"timeout message");
                waitingMessage = false;
                currentFase = Begin;
                
            }
            else if ( automaticUpdatePose ) {
                updatePose();
            }
            
			const int numByte = serial.available();
			if (numByte > 0) {

				uint8_t message[1024];
				serial.readBytes(message,numByte);
                for ( int i=0; i < numByte ; i++ ){
                    messages.insert(messages.end(), message[i]);
                }
                
            }
            
            while ( messages.size() > 0 ){
                uint8_t byte = messages.front();

                switch ( currentFase ){
                    case Begin:{
                        if ( byte == 0xAA){
                            currentFase = Header;
                        }
                        else{
                            
                            currentFase = Begin;
                            
                        }
                        break;
                    }

                    case Header:{
                        if ( byte == 0xAA){
                            currentFase = PayloadLenght;
                        }
                        else{
                           
                            currentFase = Begin;
                            
                        }
                        break;
                    }
                    case PayloadLenght:{
                        
                        payloadLenght = byte;
                        currentFase = cmdId;
                        
                        break;
                        
                    }
                    case cmdId:{
                        
                        idProtocol = byte;
						//ofLog(OF_LOG_VERBOSE, ofToString(int(idProtocol)));
                        currentFase = Ctrl;
                        break;
                    }
                    case Ctrl:{
                        ctrl = byte;
                        rw = byte & 0x01;
                        isQueued = (byte >> 1) & 0x01;
						if (payloadLenght == 2) {
							currentFase = Checksum;
						}
						else {
							currentFase = Params;
						}
                        params.clear();
                        break;
                    }
                        
                    case Params:{
                        if ( params.size() < payloadLenght - 2){
                            params.push_back(byte);
                            if ( params.size() == payloadLenght -2){
                                elaborateParams(idProtocol, params);
								currentFase = Checksum;
                            }
                        }
                        break;
                    }
                    case Checksum:{
                        
                        checksum = byte;
                        
                        uint8_t verifiedChecksum = 0;
                        verifiedChecksum += idProtocol;
                        verifiedChecksum += ctrl;
                        for ( int i=0; i < params.size(); i++)
                            verifiedChecksum += params[i];
                        
                        verifiedChecksum += checksum;
                        
                        if ( verifiedChecksum != 0)
                            ofLog(OF_LOG_ERROR,"checksum error");
                        currentFase = Begin;
						//ofLog(OF_LOG_VERBOSE, "checksum");
                        break;
                    }
                    case Error:{
                        ofLog(OF_LOG_ERROR,"error returning message");
                        currentFase = Begin;
                        break;
                    }
                        
                }
                messages.pop_front();

            }
            
        }
	}
}
