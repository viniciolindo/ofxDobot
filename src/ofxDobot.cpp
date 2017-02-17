#include "ofxDobot.h"


ofxDobot::ofxDobot(){

	noConnection = "Dobot not connected";

}

bool ofxDobot::setup(string serialName) {

	this->serialName = serialName;

	connected = serial.setup(serialName, 115200);

	waitingMessage = false;

	startThread();

	queuedCmdIndex = 0;
	queuedLeftSpace = 32;
	lastTimeMessage = 0;

	return connected;

}




bool ofxDobot::load(string fileName) {

	cout << " file name = " << fileName << endl;
	ofFile file;
	bool exist = file.open(ofToDataPath(fileName));
	if ( exist ){
		if (file.getExtension() == "xml") {
			file.close();
			isXml = true;
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
			isXml = false;

			buffer = file.readToBuffer();
			string textFile = buffer.getText();
			lines = ofSplitString(textFile, "\n");

		}
		else{
			ofLog(OF_LOG_ERROR,fileName +" extension not compatible");
		}
}
else{
	ofLog(OF_LOG_ERROR,fileName +" don't exist");

}


return exist;

}

void ofxDobot::play() {

	setQueuedCmdStartExec();

}

void ofxDobot::stop() {

	setQueuedCmdStopExec();

}

void ofxDobot::clear() {

	setQueuedCmdClear();

}

void ofxDobot::update(){

	if (queuedLeftSpace > 0 && ofGetElapsedTimef() - lastTimeMessage > 1 ){


		if ( isXml && 	timeline.tagExists("row" + ofToString(rowIndex))){

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
			double timePause = timeline.getValue("item_6", 0);

			setPTPCmd((ptpMode)type, x, y, z, r);



			if (timePause > 0) {
				WAITCmd waitCmd;
				waitCmd.timeout = timePause * 1000;
				setWAITCmd(waitCmd);
			}

			rowIndex++;
			timeline.popTag();
			lastTimeMessage = ofGetElapsedTimef();
		}
		else if (!isXml && rowIndex < lines.size() ) {

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
			lastTimeMessage = ofGetElapsedTimef();
		}
	}
	else if (queuedLeftSpace == 0 && ofGetElapsedTimef() - lastTimeMessage > 1) {

		getQueuedCmdLeftSpace();
		lastTimeMessage = ofGetElapsedTimef();

	}




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

	if (connected) {
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
			return nullPose;

		}
		else {

			waitingMessage = true;
			while (waitingMessage) {
				yield();
			}

			return pose;
		}
	}

	else {
		ofLog(OF_LOG_ERROR, noConnection);
		return nullPose;
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

		}
		else {
			waitingMessage = true;
			int time = 0;
			while (waitingMessage) {
				yield();
				time += ofGetElapsedTimeMillis();
				if (time > TIMEOUT) {
					waitingMessage = false;
					ofLog(OF_LOG_ERROR, " queued left space timeout error");
				}
			}
		}

	}

	else {
		ofLog(OF_LOG_ERROR, noConnection);
	}

	return queuedLeftSpace;

}

void ofxDobot::threadedFunction() {


	while (isThreadRunning()) {

		sleep(30);
		if (connected) {
			int numByte = serial.available();
			if (numByte > 0) {

				uint8_t message[128];
				serial.readBytes(message,numByte);
				if (message[0] == 0xAA && message[1] == 0xAA) {
					uint8_t payloadLenght = message[2];

					uint8_t checksumArrived = message[payloadLenght + 3];
					uint8_t id = message[3];

					uint8_t rw = message[4] & 0x01;
					uint8_t isQueued = (message[4] >> 1) & 0x01;

					uint8_t checksum = 0;


					for (int i = 0; i < payloadLenght+1; i++) {
						checksum += message[3 + i];
					}
					if ( checksum == 0) {

						ofLog(OF_LOG_VERBOSE,"message corrected");

						if (id == DeviceSN) {
							serialNumber.clear();
							for (int i = 0; i < payloadLenght - 2; i++) {
								const char c = message[5 + i];
								serialNumber.append(&c, 1);
							}

							waitingMessage = false;

						}
						else if ( id == DeviceName ){
							name.clear();
							for (int i = 0; i < payloadLenght - 2; i++) {
								const char c = message[5 + i];
								name.append(&c, 1);
							}

							waitingMessage = false;

						}

						else if (id == GetPose) {

							ofLog(OF_LOG_VERBOSE, "Get Pose");

							memcpy(&pose.x, &message[5], 4);
							memcpy(&pose.y, &message[9], 4);
							memcpy(&pose.z, &message[13], 4);
							memcpy(&pose.r, &message[17], 4);
							memcpy(pose.jointAngle, &message[21], 16);
							waitingMessage = false;
						}

						else if (id == ResetPose) {
							ofLog(OF_LOG_VERBOSE, "Reset Pose");
						}

						else if (id == GetAlarms) {
							ofLog(OF_LOG_VERBOSE, "Get Alarms State");
							for (int i = 0; i < 16; i++) {
								alarmsState[i] = message[5 + i];
							}
							waitingMessage = false;
						}
						else if (id == ClearAllAlarmsState) {
							ofLog(OF_LOG_VERBOSE, "Clear alarms state");
						}


						else if (id == HomeParams) {
							ofLog(OF_LOG_VERBOSE, "Home Params");
							if (message[4] == 11) {
								memcpy(&queuedCmdIndex, &message[5], 8);
								ofLog(OF_LOG_VERBOSE, " queued index = " + ofToString(queuedCmdIndex));
							}
						}

						else if (id == HomeCmd) {
							ofLog(OF_LOG_VERBOSE, "Home Cmd");
							memcpy(&queuedCmdIndex, &message[5], 8);
							ofLog(OF_LOG_VERBOSE, " queued index = " + ofToString(queuedCmdIndex));
							queuedLeftSpace = getQueuedCmdLeftSpace();

						}
						else if (id == SETGETJOGJointParams) {
							if (rw) {
								if (isQueued) {
									memcpy(&queuedCmdIndex, &message[5], 8);
									ofLog(OF_LOG_VERBOSE, "SETGETJOGJointParams arrived queued index = " + ofToString(queuedCmdIndex));
								}
								else {
									ofLog(OF_LOG_VERBOSE, "SETGETJOGJointParams arrived");
								}
							}
							else {
								memcpy(jogJointParams.velocity, &message[5], 16);
								memcpy(jogJointParams.acceleration, &message[21], 16);
								ofLog(OF_LOG_VERBOSE, "get JOGjointparams");
								waitingMessage = false;
							}
						}
						else if (id == SETGETJOGCoordinateParams) {
							if (rw) {
								if (isQueued) {
									memcpy(&queuedCmdIndex, &message[5], 8);
									ofLog(OF_LOG_VERBOSE, "SETGETJOGCoordinateParams arrived queued index = " + ofToString(queuedCmdIndex));
								}
								else {
									ofLog(OF_LOG_VERBOSE, "SETGETJOGCoordinateParams arrived" );
								}
							}
							else {
								memcpy(jogCoordinateParams.velocity, &message[5], 16);
								memcpy(jogCoordinateParams.acceleration, &message[21], 16);
								ofLog(OF_LOG_VERBOSE, "get JOGjointparams");
								waitingMessage = false;
							}
						}
						else if (id == SETGETJOGCommonParams) {
							if (rw) {
								if (isQueued) {
									memcpy(&queuedCmdIndex, &message[5], 8);
									ofLog(OF_LOG_VERBOSE, "SETGETJOGCommonParams arrived queued index = " + ofToString(queuedCmdIndex));
								}
								else {
									ofLog(OF_LOG_VERBOSE, "SETGETJOGCommonParams arrived");
								}
							}
							else {
								memcpy(&jogCommonParams.velocityRatio, &message[5], 4);
								memcpy(&jogCommonParams.accelerationRatio, &message[9], 4);
								ofLog(OF_LOG_VERBOSE, "get JOGjointparams");
								waitingMessage = false;
							}

						}


						else if (id == SetJOGCmd) {

							memcpy(&queuedCmdIndex, &message[5], 8);
							ofLog(OF_LOG_VERBOSE, "set Jogl arrived queued index = " + ofToString(queuedCmdIndex));

						}
						else if (id == PTPJointParams) {
							ofLog(OF_LOG_VERBOSE, "PTP JointParams");
						}
						else if (id == PTPCoordinateParams) {
							ofLog(OF_LOG_VERBOSE, "PTP coordinate params");
						}

						else if (id == PTPJumpParams) {
							ofLog(OF_LOG_VERBOSE, "PTP Jump params");
						}
						else if (id == PTPCommonParams) {
							ofLog(OF_LOG_VERBOSE, "PTP common params");
						}
						else if (id == PTPCmd) {
							ofLog(OF_LOG_VERBOSE, "PTPCmd");
							memcpy(&queuedCmdIndex, &message[5], 8);
							ofLog(OF_LOG_VERBOSE, "queued index = " + ofToString(queuedCmdIndex));
							queuedLeftSpace = getQueuedCmdLeftSpace();
						}

						else if (id == SETGETCPParams) {

							if (rw) {
								ofLog(OF_LOG_VERBOSE, "SETCPParams");
								if (isQueued) {
									memcpy(&queuedCmdIndex, &message[5], 8);
									ofLog(OF_LOG_VERBOSE, "Queued index = " + ofToString(queuedCmdIndex));
								}

							}
							else {
								ofLog(OF_LOG_VERBOSE, "GETCPParams");
								memcpy(&cpParams.planAcc, &message[5], 4);
								memcpy(&cpParams.junctionVel, &message[9], 4);
								cpParams.realTimeTrack = message[17];
								if ( cpParams.realTimeTrack )
									memcpy(&cpParams.period, &message[13], 4);
								else
									memcpy(&cpParams.acc, &message[13], 4);

								waitingMessage = false;
							}

						}
						else if (id == SetCPCmd) {

							memcpy(&queuedCmdIndex, &message[5], 8);
							ofLog(OF_LOG_VERBOSE, "SetCPCmd Queued index = " + ofToString(queuedCmdIndex));


						}
						else if (id == SetWAITCmd) {
							memcpy(&queuedCmdIndex, &message[5], 8);
							ofLog(OF_LOG_VERBOSE, "SetWAITCmd Queued index = " + ofToString(queuedCmdIndex));
						}

						else if (id == AngleSensorStaticError) {
							if (rw) {
								ofLog(OF_LOG_VERBOSE, "Set Angle Static Error");
							}
							else {
								ofLog(OF_LOG_VERBOSE, "Get Angle Static Error");
								memcpy(&armAngleError.rearArmAngleError, &message[5], 4);
								memcpy(&armAngleError.frontArmAngleError, &message[9], 4);
								waitingMessage = false;
							}
						}


						else if (id == SetQueuedCmdStartExec) {
							ofLog(OF_LOG_VERBOSE, "SetQueuedCmdStartExec");
						}
						else if (id == SetQueuedCmdStopExec) {
							ofLog(OF_LOG_VERBOSE, "SetQueuedCmdStopExec");
						}
						else if (id == SetQueuedCmdClear) {
							ofLog(OF_LOG_VERBOSE, "SetQueuedCmdClear");
						}
						else if (id == GetQueuedCmdCurrentIndex) {
							ofLog(OF_LOG_VERBOSE, "GetQueuedCmdCurrentIndex");
							memcpy(&queuedCmdIndex, &message[5], 8);
							ofLog(OF_LOG_VERBOSE, "queued index = " + ofToString(queuedCmdIndex));
							waitingMessage = false;
						}
						else if (id == GetQueuedCmdLeftSpace) {
							ofLog(OF_LOG_VERBOSE, "GetQueuedCmdLeftSpace");
							memcpy(&queuedLeftSpace, &message[5], 4);
							ofLog(OF_LOG_VERBOSE, "queued left space = " + ofToString(queuedLeftSpace));
							waitingMessage = false;
						}





					}
					else {
						ofLog(OF_LOG_ERROR, "checksum error");
						waitingMessage = false;
					}
				}
				else {
					ofLog(OF_LOG_ERROR, "non conformed message");
					waitingMessage = false;
				}

			}
			else {
				//ofLog(OF_LOG_ERROR, "non returned message");
				//waitingMessage = false;
			}
		}
	}
}
