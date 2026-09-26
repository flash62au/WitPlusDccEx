/* -*- c++ -*-
 *
 * WitPlusDccEx
 *
 * This package implements either a WiThrottle protocol connection,
 * or a DCC-EX native protocol connection to allow a device to communicate 
 * with a JMRI server or other WiThrottleProtocol device (like the Digitrax 
 * LNWI), or DCC-EX EX-CommandStation.
 *
 * Copyright © 2025 Peter Akers
 * Original code - Copyright © 2018-2019 Blue Knobby Systems Inc.
 *
 * This work is licensed under the Creative Commons Attribution-ShareAlike
 * 4.0 International License. To view a copy of this license, visit
 * http://creativecommons.org/licenses/by-sa/4.0/ or send a letter to
 * Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.
 *
 * Attribution — You must give appropriate credit, provide a link to the
 * license, and indicate if changes were made. You may do so in any
 * reasonable manner, but not in any way that suggests the licensor
 * endorses you or your use.
 *
 * ShareAlike — If you remix, transform, or build upon the material, you
 * must distribute your contributions under the same license as the
 * original.
 *
 * All other rights reserved.
 *
 */

//#include <ArduinoTime.h>
//#include <TimeLib.h>
#include <vector>  //https://github.com/arduino-libraries/Arduino_AVRSTL

#include "WitPlusDccEx.h"

static const int MIN_SPEED = 0;
static const int MAX_SPEED = 126;
static const char *rosterSegmentDesc[] = {"Name", "Address", "Length"};

WitPlusDccEx::WitPlusDccEx(bool server) {

	// store server/client
    this->server = server;
		
	// init streams
    stream = &nullStream;
	console = &nullStream;
}

// DONE - OK
// init the WitPlusDccEx instance after connection to the SSID
void WitPlusDccEx::init() {
    if (logLevel>0) console->println("WiT+DccEx:: init()");
    
	// allocate input buffer and init position variable
	memset(inputbuffer, 0, sizeof(inputbuffer));
	nextChar = 0;

    // output buffer
    outboundBuffer = "";
    outboundCmdsTimeLastSent = millis();
	
	// init heartbeat
	heartbeatTimer = millis();
    heartbeatPeriod = 10;
    timeLastLocoAcquired = 0;
                           
	
	// init fasttime
	fastTimeTimer = millis();
    currentFastTime = 0.0;
    currentFastTimeRate = 0.0;


	// init global variables
    for (int multiThrottleIndex=0; multiThrottleIndex<6; multiThrottleIndex++) {
        locomotiveSelected[multiThrottleIndex] = false;
        currentSpeed[multiThrottleIndex] = 0;
        speedSteps[multiThrottleIndex] = 1;  //1=128 steps
        currentDirection[multiThrottleIndex] = Forward;
        locomotives[multiThrottleIndex].resize(0);
        locomotivesFacing[multiThrottleIndex].resize(0);

        // if (isDccExServer()) {
        //     for (int locoIndex=0; locoIndex<10; locoIndex++) {
        //         dccExThrottleLocos[multiThrottleIndex][locoIndex] = 0;
        //         dccExThrottleLocoDirections[multiThrottleIndex][locoIndex] = Forward;
        //     }
        //     for (int functionIndex=0; functionIndex<32; functionIndex++) {
        //         dccExThrottleFunctionIsLatching[multiThrottleIndex][functionIndex] = false;
        //     }
        // }
        lastSpeedCommandSent[multiThrottleIndex] = 0;
    }
    
    for (int trackIndex=0; trackIndex<MAX_TRACKS; trackIndex++) {
        trackType[trackIndex] = TRACK_TYPE_NONE;
        trackPower[trackIndex] = PowerUnknown;
    }

    //last Response time
    lastServerResponseTime = millis() /1000;
	
	// init change flags
    resetChangeFlags();

    if (logLevel>0) console->println("init(): end");
}

// NO CHANGE
// Set the delegate instance for callbasks
void WitPlusDccEx::setDelegate(WitPlusDccExDelegate *delegate) {
	
    this->delegate = delegate;
}

// NO CHANGE
// Set the Stream used for logging
void WitPlusDccEx::setLogStream(Stream *console) {
	
    this->console = console;
}

// NO CHANGE
// Set the level of logging
void WitPlusDccEx::setLogLevel(int level) {
    logLevel = level;
}

//NO CHANGE
void WitPlusDccEx::resetChangeFlags() {
    clockChanged = false;
    heartbeatChanged = false;
}

// used for DCC-EX
int WitPlusDccEx::getNumberOfParameters(char *c, int startPosition, int len) {
    return getNumberOfParameters(c, startPosition, len, " ");
}

// used for DCC-EX
int WitPlusDccEx::getNumberOfParameters(char *c, int startPosition, int len, String separator) {
    String s(c);
    s = s.substring(0, len-1);
    // if (logLevel>1) { console->print("WiT+DccEx:: getNumberOfParameter()"); console->print(s);console->println("~");}

    // loop
    int entries = 0;
    boolean entryFound = true;
    int entryStartPosition = startPosition;
    if (sizeof(c) <= 3) entryFound = false;

    while (entryFound) {
        entries++;

        int entrySeparatorPosition;
        if (s.charAt(entryStartPosition) == '"') {
            entrySeparatorPosition = s.indexOf("\"", entryStartPosition + 1) + 1;
        } else {
            entrySeparatorPosition = s.indexOf(separator, entryStartPosition);
        }
        
        if (entrySeparatorPosition==-1) entrySeparatorPosition = s.length();
        String entry = s.substring(entryStartPosition, entrySeparatorPosition);

        // if (logLevel>1) { console->print("WiT+DccEx:: getParameter() Parameter: "); console->print(entries); console->print(": "); console->println(entry); }
        
        entryStartPosition = entrySeparatorPosition + 1;

        if (entryStartPosition >= s.length()) entryFound = false;
    }

    console->print("WiT+DccEx:: getNumberOfParameters: count: "); console->println(entries);

    return entries;
}

String WitPlusDccEx::getLocoStringFromDccAddress(String dccAddress) {
    int num = 0;
    try {
        num = dccAddress.toInt();
    } catch (const std::invalid_argument& e) {
        return "";
    } catch (const std::out_of_range& e) {
        // Handle case where the number is too big for a standard int
        return "";
    }

    if (num<=127) {
        return "S" + dccAddress;
    }
    return "L" + dccAddress;
}

// DONE - used for DCC-EX
String WitPlusDccEx::getParameter(char *c, int startPosition, int len, int parameterNo) {
    return getParameter(c, startPosition, len, " ", parameterNo);
}
// DONE - used for DCC-EX
String WitPlusDccEx::getParameter(char *c, int startPosition, int len, String separator, int parameterNo) {
    String s(c);
    s = s.substring(0, len-1);
    // if (logLevel>1) { console->print("WiT+DccEx:: getParameter(): «"); console->print(s); console->println("»");}

    // loop
    int entries = -1;
    boolean entryFound = true;
    int entryStartPosition = startPosition;
    if (sizeof(c) <= 3) entryFound = false;

    while (entryFound) {
        entries++;

        int entrySeparatorPosition;
        if (s.charAt(entryStartPosition) == '"') {
            entrySeparatorPosition = s.indexOf("\"", entryStartPosition + 1) + 1;
        } else {
            entrySeparatorPosition = s.indexOf(separator, entryStartPosition);
        }
        
        if (entrySeparatorPosition==-1) entrySeparatorPosition = s.length();
        String entry = s.substring(entryStartPosition, entrySeparatorPosition);

        if (entries == parameterNo) {
            // if (logLevel>2) { console->print("WiT+DccEx:: getParameter() Parameter: "); console->print(parameterNo); console->print(": "); console->println(entry); }
            return entry;
        }
        
        entryStartPosition = entrySeparatorPosition + 1;
        
        // if (logLevel>3) {
        //     console->print("WiT+DccEx:: getParameter() start: "); console->println(entryStartPosition);
        //     console->print("WiT+DccEx:: getParameter() len:   "); console->println(s.length());
        // }

        if (entryStartPosition >= s.length()) entryFound = false;
    }

    return "";
}

// // Finds the Nth expression (0-indexed) in a char array, respecting quotes.
// std::string WitPlusDccEx::getNthExpression(const char* str, int targetIndex, char delimiter = ' ') {
//     if (!str) return "";

//     int currentIndex = 0;
//     std::string currentToken = "";
//     bool inDoubleQuotes = false;
//     bool inSingleQuotes = false;

//     for (int i = 0; str[i] != '\0'; ++i) {
//         char c = str[i];

//         // Handle quote toggling
//         if (c == '"' && !inSingleQuotes) {
//             inDoubleQuotes = !inDoubleQuotes;
//             continue; // Skip adding the quote character itself to the token
//         } else if (c == '\'' && !inDoubleQuotes) {
//             inSingleQuotes = !inSingleQuotes;
//             continue; // Skip adding the quote character itself to the token
//         }

//         // If we hit a delimiter outside of any quotes
//         if (c == delimiter && !inDoubleQuotes && !inSingleQuotes) {
//             if (!currentToken.empty()) {
//                 if (currentIndex == targetIndex) {
//                     return currentToken;
//                 }
//                 currentIndex++;
//                 currentToken.clear();
//             }
//         } else {
//             // Build the current token
//             currentToken += c;
//         }
//     }

//     // Check the final token after the loop ends
//     if (!currentToken.empty() && currentIndex == targetIndex) {
//         return currentToken;
//     }

//     return ""; // Return empty string if Nth expression doesn't exist
// }

// DONE - used for DCC-EX
int WitPlusDccEx::getSpeedFromSpeedByte(String speedByte) {
    // int dir = 0;
    int speed = 0;
    try {
        speed = speedByte.toInt();
    } catch (const std::out_of_range& e) {
        return -1;
    }

    if (speed >= 128) {
        speed = speed - 128;
        // dir = 1;
    }
    if (speed>1) {
        speed = speed - 1; // get around the idiotic design of the speed command
    } else {
        speed=0;
    }
    return speed;
}

// DONE - used for DCC-EX
Direction WitPlusDccEx::getDirectionFromSpeedByte(String speedByte) {
    Direction dir = Forward;
    int speed = 0;
    try {
        speed = speedByte.toInt();
    } catch (const std::out_of_range& e) {
        return Forward;
    }

    if (speed >= 128) {
        dir = Reverse;
    }
    return dir;
}

// DONE
bool WitPlusDccEx::isWiThrottleServer() {
    return serverType == WITHROTTLE_PROTOCOL ? true : false;
}

// DONE
bool WitPlusDccEx::isDccExServer() {
    return serverType == DCCEX_PROTOCOL ? true : false;
}

// DONE
bool WitPlusDccEx::isLocoInThrottle(char multiThrottle, String address) {
    if (logLevel>2) console->println("WiT+DccEx:: isLocoInThrottle()");

    int index = getLocoIndexInThrottle(multiThrottle, address);
    return index != -1;
}

// DONE
int WitPlusDccEx::getLocoIndexInThrottle(char multiThrottle, String address) {
    if (logLevel>2) { console->print("WiT+DccEx:: getLocoIndexInThrottle(): multiThrottle: "); console->print(multiThrottle); console->print(" address: "); console->println(address); }

    int multiThrottleIndex = getMultiThrottleIndex(multiThrottle);
    if (multiThrottleIndex == -1) return -1;

    for(int i=0;i<locomotives[multiThrottleIndex].size();i++) {
        if (logLevel>2) { console->print("WiT+DccEx:: getLocoIndexInThrottle(): address: "); console->println(locomotives[multiThrottleIndex][i]); }
        if (locomotives[multiThrottleIndex][i].equals(address)) {
            return i;
        }
    } 

    return -1;
}


// DONE
bool WitPlusDccEx::addLocoToThrottle(char multiThrottle, String address) {
    if (logLevel>0) console->println("WiT+DccEx:: addLocoToThrottle()");

    if (getLocoIndexInThrottle(multiThrottle, address) != -1) return false;
    
    int multiThrottleIndex = getMultiThrottleIndex(multiThrottle);

    locomotives[multiThrottleIndex].push_back(address);
    currentAddress[multiThrottleIndex] = locomotives[multiThrottleIndex].front();
    locomotivesFacing[multiThrottleIndex].push_back(Forward);
    locomotiveSelected[multiThrottleIndex] = true;
    timeLastLocoAcquired = millis();
    return true;
}

// DONE
bool WitPlusDccEx::removeLocoFromThrottle(char multiThrottle, String address) {
    if (logLevel>0) console->println("WiT+DccEx:: removeLocoFromThrottle()");

    int multiThrottleIndex = getMultiThrottleIndex(multiThrottle);

    if (address.equals(ALL_LOCOS_ON_THROTTLE)) {
        locomotives[multiThrottleIndex].clear();
        locomotivesFacing[multiThrottleIndex].clear();
    } else {
        int index = getLocoIndexInThrottle(multiThrottle, address);
        if (index== -1) return false;

        locomotives[multiThrottleIndex].erase(locomotives[multiThrottleIndex].begin()+index);
        locomotivesFacing[multiThrottleIndex].erase(locomotivesFacing[multiThrottleIndex].begin()+index);
    }

    if (locomotives[multiThrottleIndex].size()==0) { 
        locomotiveSelected[multiThrottleIndex] = false;
        currentAddress[multiThrottleIndex] = "";
    } else {        
        currentAddress[multiThrottleIndex] = locomotives[multiThrottleIndex].front();
    }

    return true;
}

// DONE
void WitPlusDccEx::setProtocol(Protocol type) {
    // True = WiThrottle Server, False = DCC-EX Native Protocol Server
    serverType = type;
}

// NO CHANGE
void WitPlusDccEx::connect(Stream *stream) {
    // init();
    // this->stream = stream;
    connect(stream, 50, true);
}

// DONE
void WitPlusDccEx::connect(Stream *stream, int delayBetweenCommandsSent) {
    if (logLevel>0) console->println("WiT+DccEx:: connect()");

    init();
    this->stream = stream;

    outboundCmdsMinimumDelay = delayBetweenCommandsSent;
    if (logLevel>0) {
        console->print("WiT+DccEx:: connect(): Outbound commands minimum delay: "); console->println(outboundCmdsMinimumDelay);
    }

    if (isDccExServer()) {
        rosterListReceived = false;
    }
}

// DONE - WiThrottle Only
void WitPlusDccEx::disconnect() {
    if (isWiThrottleServer()) {
        String command = "Q";
        sendDelayedCommand(command);
        this->stream = NULL;

    // } else { //DCC-EX - do nothing
    }
}

// DONE
void WitPlusDccEx::setDeviceName(String deviceName) {
    currentDeviceName = deviceName;
    if (isWiThrottleServer()) {
        String command = "N" + deviceName;
        sendDelayedCommand(command);

    } else { // DCC-EX
        dccExSetDeviceName();
    }
}

// DONE - WiThrottle only
void WitPlusDccEx::setDeviceID(String deviceId) {
    if (isWiThrottleServer()) {
        String command = "HU" + deviceId;
        sendDelayedCommand(command);

    // } else { //DCC-EX - do nothing
    }
}

// DONE
void WitPlusDccEx::setCommandsNeedLeadingCrLf(bool needed) {
    if (isWiThrottleServer()) {
        commandsNeedLeadingCrLf = needed;

     } else { //DCC-EX - never required
        commandsNeedLeadingCrLf = false;
    }
}

// APPEARS OK - TODO - DELAY - DCC-EX is character based
bool WitPlusDccEx::check() {
    bool changed = false;
    resetChangeFlags();

    if (stream) {
        // update the fast clock first
        changed |= checkFastTime();
        changed |= checkHeartbeat();

        while(stream->available()) {
            char b = stream->read();
            // if (logLevel>2) { console->print("WiT+DccEx:: check() : "); console->println(b); }
            if (b == NEWLINE || b==CR) {
                // server sends TWO newlines after each command, we trigger on the
                // first, and this skips the second one
                if (nextChar != 0) {
                    inputbuffer[nextChar] = 0;
                    changed |= processCommand(inputbuffer, nextChar);
                    heartbeatTimer = millis();
                }
                nextChar = 0;
            }
            else {
                inputbuffer[nextChar] = b;
                nextChar += 1;
                if (nextChar == (sizeof(inputbuffer)-1) ) {
                    inputbuffer[sizeof(inputbuffer)-1] = 0;
                    console->print("WiT+DccEx:: ERROR LINE TOO LONG: >");
                    console->print(sizeof(inputbuffer));
                    console->print(": ");
                    console->println(inputbuffer);
                    nextChar = 0;
                }
            }
        }
        sendDelayedCommand("");  // force the outbound buffer to be flushed if needed.

        return changed;

    }
    else {
        return false;
    }
}

// NO CHANGE
void WitPlusDccEx::sendCommand(String cmd) {

    if (stream) {
        // TODO - DELAY : what happens when the write fails?
        stream->println(cmd);
        if (server) {
            stream->println("");
        }
        console->print("WiT+DccEx:: ==> "); console->println(cmd);
    }
}

// NO CHANGE
void WitPlusDccEx::sendDelayedCommand(String cmd) {

    if (stream) {
        // TODO - DELAY : what happens when the write fails?

        if (cmd.length()>0) {
            outboundBuffer = outboundBuffer + cmd + '\n';
        }

        if ( (outboundBuffer.length()>0) &&((millis()-outboundCmdsTimeLastSent) > outboundCmdsMinimumDelay) ) {
            if (logLevel>1) {
                console->print("WiT+DccEx:: sendDelayedCommand() : Flushing outbound buffer - delay: "); console->print(outboundCmdsMinimumDelay); console->print(" Buffer: ");  console->println(outboundBuffer);
            }
            int end = outboundBuffer.indexOf("\n");
            String thisCmd = outboundBuffer; // default to sending the lot
            if (end>0) {
                thisCmd = outboundBuffer.substring(0,end);
                end++;
                outboundBuffer = outboundBuffer.substring(end);
                if (outboundBuffer.length()>0) {
                    if (logLevel>1) {
                        console->print("WiT+DccEx:: sendDelayedCommand() : deferring cmds: "); console->println(outboundCmdsMinimumDelay); 
                        console->println("WiT+DccEx:: Buffer: ");  console->println(outboundBuffer);
                    }
                }
            } else if (end==0) {
                thisCmd = "";
                outboundBuffer = outboundBuffer.substring(end+1);
            } else {
                thisCmd = outboundBuffer;
                outboundBuffer = "";
            }

            if (thisCmd.length()>0) {
                outboundCmdsTimeLastSent = millis();
                if (commandsNeedLeadingCrLf) {
                    stream->write(0x0D);
                    stream->write(0x0A);
                }
                stream->println(thisCmd);

                if (server) {
                    stream->println("");
                }
                if (logLevel>0) {
                    console->print("WiT+DccEx:: ==> "); console->print(thisCmd);
                    console->print(" ("); console->print(millis()); console->println(")");
                }
            }
        }
    }
}

// NO CHANGE
bool WitPlusDccEx::checkFastTime() {
	
    bool changed = true;
    
	// check if a second has passed
	if ((millis() - fastTimeTimer) > 1000) { 
        
		fastTimeTimer = millis();
        
		// no FastTime
		if (currentFastTimeRate == 0.0) clockChanged = false;
		
		// FastTime, update accordingly to rate
        else {
            currentFastTime += currentFastTimeRate;
            clockChanged = true;
        }
    }

    return changed;
}


/*int
WitPlusDccEx::fastTimeHours()
{
    time_t now = (time_t) currentFastTime;
    return hour(now);
}


int
WitPlusDccEx::fastTimeMinutes()
{
    time_t now = (time_t) currentFastTime;
    return minute(now);
}*/


// NO CHANGE
double WitPlusDccEx::getCurrentFastTime() {

	return currentFastTime;
}

// NO CHANGE
float WitPlusDccEx::getFastTimeRate() {
    return currentFastTimeRate;
}

// NO CHANGE - WiThrottle only
bool WitPlusDccEx::processLocomotiveAction(char multiThrottle, char *c, int len) {
    if (logLevel>0) console->println("WiT+DccEx:: processLocomotiveAction()");

    int multiThrottleIndex = getMultiThrottleIndex(multiThrottle);
    String remainder(c);  // the leading "MTA" was not passed to this method

    if (logLevel>0) console->printf("WiT+DccEx:: processLocomotiveAction(): remainder at first is %s\n", remainder.c_str());

    if (currentAddress[multiThrottleIndex].equals("")) {
        if (logLevel>0) console->printf("WiT+DccEx:: skipping due to no selected address\n");
        return true;
    }
    else {
        if (logLevel>1) console->printf("WiT+DccEx:: currentAddress is '%s'\n", currentAddress[multiThrottleIndex].c_str());
    }

    bool isLeadOrAll = true;
    String address = "";
    String addrCheck = currentAddress[multiThrottleIndex] + PROPERTY_SEPARATOR;
    String allCheck = "*";
    allCheck.concat(PROPERTY_SEPARATOR);
    if (remainder.startsWith(addrCheck)) {
        remainder.remove(0, addrCheck.length());
    } else if (remainder.startsWith(allCheck)) {
        remainder.remove(0, allCheck.length());
    } else {
        int p = remainder.indexOf(PROPERTY_SEPARATOR);
        if (p > 0) { // non-lead loco
            address = remainder.substring(0, p);
            addrCheck = address + PROPERTY_SEPARATOR;
            remainder.remove(0, addrCheck.length());
            isLeadOrAll = false;
        }
    }

    if (logLevel>1) console->printf("WiT+DccEx:: processLocomotiveAction: after separator is %s\n", remainder.c_str());

    if (remainder.length() > 0) {
        char action = remainder[0];

        if (isLeadOrAll) {
            switch (action) {
                case 'F':
                    if (logLevel>1) console->printf("WiT+DccEx:: processing function state\n");
                    processFunctionState(multiThrottle, remainder);
                    break;
                case 'V':
                    processSpeed(multiThrottle, remainder);
                    break;
                case 's':
                    processSpeedSteps(multiThrottle, remainder);
                    break;
                case 'R':
                    processDirection(multiThrottle, remainder);
                    break;
                default:
                    if (logLevel>0) console->printf("WiT+DccEx:: unrecognized action '%c'\n", action);
                    // no processing on unrecognized actions
                    break;
            }
        } else { // non-lead loco
            if (logLevel>0) console->printf("WiT+DccEx:: Non-lead loco action '%c'\n", action);
            switch (action) {
                case 'F':
                case 'V':
                case 's':
                    break;
                case 'R':
                    processDirection(multiThrottle, address, remainder);
                    break;
                default:
                    if (logLevel>0) console->printf("WiT+DccEx:: unrecognized action '%c'\n", action);
                    // no processing on unrecognized actions
                    break;
            }
        }
        return true;
    }
    else {
        if (logLevel>0) console->printf("WiT+DccEx:: insufficient action to process\n");
        return false;
    }
}

// NO CHANGE - WiThrottle only
bool WitPlusDccEx::processRosterFunctionList(char multiThrottle, char *c, int len) {
    if (logLevel>0) console->println("WiT+DccEx:: processRosterFunctionList()");

    int multiThrottleIndex = getMultiThrottleIndex(multiThrottle);
    String remainder(c);  // the leading "MTL" was not passed to this method

    if (logLevel>0) console->printf("WiT+DccEx:: processRosterFunctionList(): remainder at first is %s\n", remainder.c_str());

    if (currentAddress[multiThrottleIndex].equals("")) {
        if (logLevel>0) console->printf("WiT+DccEx:: skipping due to no selected address\n");
        return true;
    }
    else {
        if (logLevel>0) console->printf("WiT+DccEx:: currentAddress is '%s'\n", currentAddress[multiThrottleIndex].c_str());
    }

    String addrCheck = currentAddress[multiThrottleIndex] + PROPERTY_SEPARATOR;
    String allCheck = "*";
    allCheck.concat(PROPERTY_SEPARATOR);
    if (remainder.startsWith(addrCheck)) {
        remainder.remove(0, addrCheck.length());
    }
    else if (remainder.startsWith(allCheck)) {
        remainder.remove(0, allCheck.length());
    }

    if (logLevel>1) console->printf("WiT+DccEx:: processRosterFunctionList(): after separator is %s\n", remainder.c_str());

    if (remainder.length() > 0) {
        char action = remainder[0];

        if (action == ']') {
            processRosterFunctionListEntries(multiThrottle, remainder);
        } else {
            if (logLevel>0) console->printf("WiT+DccEx:: unrecognized L action '%c'\n", action);
            // no processing on unrecognized actions
        }
        return true;
    }
    else {
        if (logLevel>0) console->printf("WiT+DccEx:: insufficient action to process\n");
        return false;
    }
}

// DONE
bool WitPlusDccEx::processCommand(char *c, int len) {
    bool changed = false;

    if (logLevel>0) {
        console->print("WiT+DccEx:: <== ");
        console->println(c);
    }

    lastServerResponseTime = millis()/1000;

    // we regularly get this string as part of the data sent
    // by a Digitrax LnWi.  Remove it, and try again.
    const char *ignoreThisGarbage = "AT+CIPSENDBUF=";
    while (strncmp(c, ignoreThisGarbage, strlen(ignoreThisGarbage)) == 0) {
        if (logLevel>0) console->printf("WiT+DccEx:: removed one instance of %s\n", ignoreThisGarbage);
        c += strlen(ignoreThisGarbage);
        changed = true;
    }

    if (changed) {
        if (logLevel>0) console->printf("WiT+DccEx:: input string is now: '%s'\n", c);
    }


    if (isWiThrottleServer()) {
            
        if (len > 3 && c[0]=='P' && c[1]=='F' && c[2]=='T') {
            return processFastTime(c+3, len-3);
        }
        else if (len > 3 && c[0]=='P' && c[1]=='P' && c[2]=='A') {
            processTrackPower(c+3, len-3);
            return true;
        }
        else if (len > 1 && c[0]=='*') {
            return processHeartbeat(c+1, len-1);
        }
        else if (len > 2 && c[0]=='V' && c[1]=='N') {
            processProtocolVersion(c+2, len-2);
            return true;
        }
        else if (len > 2 && c[0]=='H' && c[1]=='T') {
            processServerType(c+2, len-2);
            return true;
        }
        else if (len > 2 && c[0]=='H' && c[1]=='t') {
            processServerDescription(c+2, len-2);
            return true;
        }	
        else if (len > 2 && c[0]=='H' && c[1]=='M') {
            processAlert(c+2, len-2);
            return true;
        }	
        else if (len > 2 && c[0]=='H' && c[1]=='m') {
            processMessage(c+2, len-2);
            return true;
        }	
        else if (len > 2 && c[0]=='P' && c[1]=='W') {
            processWebPort(c+2, len-2);
            return true;
        }
        else if (len > 2 && c[0]=='R' && c[1]=='L') {
            processRosterList(c+2, len-2);
            return true;
        }	
        else if (len > 3 && c[0]=='P' && c[1]=='T' && c[2]=='L') {
            processTurnoutList(c+2, len-2);
            return true;
        }	
        else if (len > 3 && c[0]=='P' && c[1]=='R' && c[2]=='L') {
            processRouteList(c+2, len-2);
            return true;
        }	
        else if (len > 6 && c[0]=='M' && c[2]=='S') {
            processStealNeeded(c[1], c+3, len-3);
            return true;
        }
        else if (len > 6 && c[0]=='M' && (c[2]=='+' || c[2]=='-')) {
            // we want to make sure the + or - is passed in as part of the string to process
            processAddRemove(c[1], c+2, len-2);
            return true;
        }
        else if (len > 8 && c[0]=='M' && c[2]=='A') {
            return processLocomotiveAction(c[1], c+3, len-3);
        }
        else if (len > 8 && c[0]=='M' && c[2]=='L') {
            return processRosterFunctionList(c[1], c+3, len-3);
        }
        else if (len > 5 && c[0]=='P' && c[1]=='T' && c[2]=='A') {
            processTurnoutAction(c+3, len-3);
            return true;
        }
        else if (len > 4 && c[0]=='P' && c[1]=='R' && c[2]=='A') {
            processRouteAction(c+3, len-3);
            return true;
        }
        else if (len > 3 && c[0]=='A' && c[1]=='T' && c[2]=='+') {
            // this is an AT+.... command that the LnWi sometimes emits and we
            processUnknownCommand(c, len);
            // ignore these commands altogether
        }
        else {
            if (logLevel>0) console->printf("WiT+DccEx:: unknown command '%s'\n", c);
            processUnknownCommand(c, len);
            // all other commands are explicitly ignored
        }

    } else { // DCC-EX

        if (len < 3) {
            if (logLevel>0) console->printf("WiT+DccEx:: unknown DCC-EX command '%s'\n", c);
            processUnknownCommand(c, len);
            return false;
        }

        // currently this can only accept single commands per line
        // ignores the first and last char - '<" and '>'
        switch (c[1]) {
            case 'i': // Command Station Information
                dccExProcessCommandStationInfo(c+1, len-3);
                return true;

            case '-': // force Drop Loco
                // not currently supported
                processUnknownCommand(c, len);
                return false;

            case 'l': // dccExProcessLocos
                dccExProcessLocos(c+1, len-1);
                return true;

            case 'r': // 
                // response from a request for a loco id (the Drive away feature, and also the Address read)
                // or response from a CV write
                dccExProcessRequestLocoId(c+1, len-3);
                return true;

            case 'p': // power response
                dccExProcessPower(c+1, len-1);
                return true;

            case 'j': // //roster, turnouts / routes lists

                if (c[2] == 'T') { // turnouts / points
                    dccExProcessTurnouts(c+3, len-3);
                } else if (c[2] == 'A') { // automation/routes
                    dccExProcessRoutes(c+3, len-3);
                } else if (c[2] == 'B') { // automation/route update (Inactive, Active, Hidden, Caption)
                    dccExProcessRouteUpdate(c+3, len-3);
                } else if (c[2] == 'R') { // roster
                    dccExProcessRoster(c+3, len-3);
                } else if (c[2] == 'C') { // fastclock
                    dccExProcessFastClock(c+3, len-3);
                } else { //
                    if (logLevel>0) console->printf("WiT+DccEx:: unknown DCC-EX command '%s'\n", c);
                    processUnknownCommand(c, len);
                    return false;
                }  

                return true;

            case 'H': // Turnout/points change
                dccExProcessTurnoutUpdate(c+3, len-3);
                return true;

            case 'v': // response from a request a CV value
                // not currently supported
                processUnknownCommand(c, len);
                return false;

            case 'w': // response from an address write or other CV write
                // not currently supported
                processUnknownCommand(c, len);
                return false;

            case '=': //  Track Manager response
                dccExProcessTrackManager(c, len);
                return true;

            case 'm': // alert / info message sent from server to throttle
                processMessage(c+1, len-3);
                return true;

            case '!': // estop
                dccExProcessEmergencyStop(c+1, len-3);
                return true;

            case 'Q': // active sensor
            case 'q': // inactive sensor
                // not currently supported
                processUnknownCommand(c, len);
                return false;

            case 'X': // error
                if (len==3) return true; // <X> just ignore it
                processUnknownCommand(c, len);
                return false;

            case '^': // in-command-station consists
                // not currently supported
                processUnknownCommand(c, len);
                return false;

                case '#': // number of supported cabs
                // not currently supported
                // used as the defacto heartbeat, so ignore it
                // processUnknownCommand(c, len);
                return true;

            default:
                if (logLevel>0) console->printf("WiT+DccEx:: unknown DCC-EX command '%s'\n", c);
                processUnknownCommand(c, len);
                break;
        }

    }
    return false;
}


// DONE - No Change
void WitPlusDccEx::setCurrentFastTime(const String& s) {
    int t = s.toInt();
    if (currentFastTime == 0.0) {
        if (logLevel>0) { console->print("WiT+DccEx:: set fast time to "); console->println(t); }
    }
    else {
        if (logLevel>0) {
            console->print("WiT+DccEx:: updating fast time (should be "); console->print(t);
            console->print(" is "); console->print(currentFastTime);  console->println(")");
            console->printf("currentTime is %ld\n", millis());
        }
    }
    currentFastTime = t;
}


// NO Change - WiThrottle only
bool WitPlusDccEx::processFastTime(char *c, int len) {
    // keep this style -- I don't validate the settings and syntax
    // as well as I could, so someday we might return false

    bool changed = false;

    String s(c);

    int p = s.indexOf(PROPERTY_SEPARATOR);
    if (p > 0) {
        String timeval = s.substring(0, p);
        String rate    = s.substring(p+3);

        setCurrentFastTime(timeval);
        currentFastTimeRate = rate.toFloat();
        if (logLevel>0) { console->print("WiT+DccEx:: set clock rate to "); console->println(currentFastTimeRate); }
        changed = true;
        clockChanged = true;
    }
    else {
        setCurrentFastTime(s);
        changed = true;
    }

    return changed;
}


// NO CHANGE - WiThrottle only
bool WitPlusDccEx::processHeartbeat(char *c, int len) {
    if (logLevel>2) console->println("WiT+DccEx:: processHeartbeat()");

    bool changed = false;
    String s(c);

    heartbeatPeriod = s.toInt();
    if (heartbeatPeriod > 0) {
        heartbeatChanged = true;
        changed = true;
        if (delegate) {
            if (delegate) delegate->heartbeatConfig(heartbeatPeriod);
        }
    }
    return changed;
}


// NO CHANGE - WiThrottle only
void WitPlusDccEx::processProtocolVersion(char *c, int len) {
    if (logLevel>0) console->println("WiT+DccEx:: processProtocolVersion()");

    if (delegate && len > 0) {
        String protocolVersion = String(c);
        if (delegate) delegate->receivedVersion(protocolVersion);
    }
}

// NO CHANGE - WiThrottle only
void WitPlusDccEx::processServerType(char *c, int len) {
    if (logLevel>0) console->println("WiT+DccEx:: processServerType()");
	
    if (delegate && len > 0) {
        String typeStr = String(c);
        if (delegate) delegate->receivedServerType(typeStr);
    }
}

// NO CHANGE - WiThrottle only
void WitPlusDccEx::processServerDescription(char *c, int len) {
    if (logLevel>0) console->println("WiT+DccEx:: processServerDescription()");
	
    if (delegate && len > 0) {

        String serverDescription;

        serverDescription = String(c);
        if (delegate) delegate->receivedServerDescription(serverDescription);
    }
}

// NO CHANGE
void WitPlusDccEx::processMessage(char *c, int len) {
    if (logLevel>1) console->println("WiT+DccEx:: processMessage()");
	
    if (delegate && len > 0) {
        String message = String(c);
        if (delegate) delegate->receivedMessage(message);
    }
}

// NO CHANGE - WiThrottle only
void WitPlusDccEx::processAlert(char *c, int len) {
    if (logLevel>1) console->println("WiT+DccEx:: processAlert()");
	
    if (delegate && len > 0) {
        String alert = String(c);
        if (delegate) delegate->receivedAlert(alert);
    }
}

// NO CHANGE - WiThrottle only
void WitPlusDccEx::processWebPort(char *c, int len) {
    if (logLevel>1) console->println("WiT+DccEx:: processWebPort()");

    if (delegate && len > 0) {
        String port_string = String(c);
        int port = port_string.toInt();

        if (delegate) delegate->receivedWebPort(port);
    }
}

// NO CHANGE - WiThrottle only
void WitPlusDccEx::processRosterList(char *c, int len) {
    if (logLevel>0) console->println("WiT+DccEx:: processRosterList()");

	String s(c);

	// get the number of entries
    int indexSeperatorPosition = s.indexOf(ENTRY_SEPARATOR,1);
	int entries = s.substring(0, indexSeperatorPosition).toInt();
	if (logLevel>0) { console->print("WiT+DccEx:: Entries in roster: "); console->println(entries);}
	
	// if set, call the delegate method
	if (delegate) delegate->receivedRosterEntries(entries);	
	
	// loop
	int entryStartPosition = 4; //ignore the first entry separator
	for(int i = 0; i < entries; i++) {
	
		// get element
		int entrySeparatorPosition = s.indexOf(ENTRY_SEPARATOR, entryStartPosition);
		String entry = s.substring(entryStartPosition, entrySeparatorPosition);
		if (logLevel>0) { console->print("WiT+DccEx:: Roster Entry: "); console->println(i + 1); }
		
		// split element in segments and parse them		
		String name;
		int address = 0;
		char length = 0;
		int segmentStartPosition = 0;
		for(int j = 0; j < 3; j++) {
		
			// get segment
			int segmentSeparatorPosition = entry.indexOf(SEGMENT_SEPARATOR, segmentStartPosition);
			String segment = entry.substring(segmentStartPosition, segmentSeparatorPosition);
			if (logLevel>0) { console->print("WiT+DccEx:: "); console->print(rosterSegmentDesc[j]); console->print(": "); console->println(segment); }
			segmentStartPosition = segmentSeparatorPosition + 3;
			
			// parse the segments
			if(j == 0) name = segment;
			else if(j == 1) address = segment.toInt();
			else if(j == 2) length = segment[0];
		}
		
		// if set, call the delegate method
		if(delegate) delegate->receivedRosterEntry(i, name, address, length);
		
		entryStartPosition = entrySeparatorPosition + 3;
	}

    if (logLevel>0) console->println("WiT+DccEx:: processRosterList(): end");
}

//NO CHANGE - WiThrottle only
void WitPlusDccEx::processTurnoutList(char *c, int len) {
    if (logLevel>0) console->println("WiT+DccEx:: processTurnoutList()");

  	String s(c);

    // loop
    int entries = -1;
    boolean entryFound = true;
	int entryStartPosition = 4; //ignore the first entry separator
    if (sizeof(c) <= 3) entryFound =false;

    while (entryFound) {
	    entries++;

		// get element
		int entrySeparatorPosition = s.indexOf(ENTRY_SEPARATOR, entryStartPosition);
        if (entrySeparatorPosition==-1) entrySeparatorPosition = s.length();
		String entry = s.substring(entryStartPosition, entrySeparatorPosition);
		if (logLevel>0) { console->print("WiT+DccEx:: Turnout Entry: "); console->println(entries + 1); }
		
		// split element in segments and parse them		
		String sysName;
		String userName;
        int state = 0;
		int segmentStartPosition = 0;
		for(int j = 0; j < 3; j++) {
		
			// get segment
			int segmentSeparatorPosition = entry.indexOf(SEGMENT_SEPARATOR, segmentStartPosition);
			String segment = entry.substring(segmentStartPosition, segmentSeparatorPosition);
			if (logLevel>0) { console->print("WiT+DccEx:: "); console->print(rosterSegmentDesc[j]); console->print(": "); console->println(segment); }
			segmentStartPosition = segmentSeparatorPosition + 3;
			
			// parse the segments
			if(j == 0) sysName = segment;
			else if(j == 1) userName = segment;
			else if(j == 2) state = segment.toInt();
		}
		
		// if set, call the delegate method
		if(delegate) delegate->receivedTurnoutEntry(entries, sysName, userName, state);
		
		entryStartPosition = entrySeparatorPosition + 3;
        if (logLevel>0) {
            console->print("WiT+DccEx:: "); console->println(entryStartPosition);
            console->print("WiT+DccEx:: "); console->println(s.length());
        }
        if (entryStartPosition >= s.length()) entryFound = false;
	}

	// get the number of entries
	if (logLevel>0) { console->print("WiT+DccEx:: Entries in Turnouts List: "); console->println(entries+1); }
	// if set, call the delegate method
	if (delegate) delegate->receivedTurnoutEntries(entries+1);	

    if (logLevel>1) console->println("WiT+DccEx:: processTurnoutList(): end");
}

// NO CHANGE - WiThrottle only
void WitPlusDccEx::processRouteList(char *c, int len) {
    if (logLevel>0) console->println("WiT+DccEx:: processRouteList()");
  	String s(c);

    // loop
    int entries = -1;
    boolean entryFound = true;
	int entryStartPosition = 4; //ignore the first entry separator
    if (sizeof(c) <= 3) entryFound =false;

    while (entryFound) {
	    entries++;

		// get element
		int entrySeparatorPosition = s.indexOf(ENTRY_SEPARATOR, entryStartPosition);
        if (entrySeparatorPosition==-1) entrySeparatorPosition = s.length();
		String entry = s.substring(entryStartPosition, entrySeparatorPosition);
		if (logLevel>0) { console->print("WiT+DccEx:: Route Entry: "); console->println(entries + 1); }
		
		// split element in segments and parse them		
		String sysName;
		String userName;
        int state = 0;
		int segmentStartPosition = 0;
		for(int j = 0; j < 3; j++) {
		
			// get segment
			int segmentSeparatorPosition = entry.indexOf(SEGMENT_SEPARATOR, segmentStartPosition);
			String segment = entry.substring(segmentStartPosition, segmentSeparatorPosition);
			if (logLevel>0) { console->print("WiT+DccEx:: "); console->print(rosterSegmentDesc[j]); console->print(": "); console->println(segment); }
			segmentStartPosition = segmentSeparatorPosition + 3;
			
			// parse the segments
			if(j == 0) sysName = segment;
			else if(j == 1) userName = segment;
			else if(j == 2) state = segment.toInt();
		}
		
		// if set, call the delegate method
		if(delegate) delegate->receivedRouteEntry(entries, sysName, userName, state);
		
		entryStartPosition = entrySeparatorPosition + 3;
        if (logLevel>0) {
            console->print("WiT+DccEx:: "); console->println(entryStartPosition);
            console->print("WiT+DccEx:: "); console->println(s.length());
        }
        if (entryStartPosition >= s.length()) entryFound = false;
	}

	// get the number of entries
	if (logLevel>0) { console->print("WiT+DccEx:: Entries in Turnouts List: "); console->println(entries+1); }
	// if set, call the delegate method
	if (delegate) delegate->receivedRouteEntries(entries+1);	

    if (logLevel>1) console->println("WiT+DccEx:: processRouteList(): end");
}

// NO CHANGE
// supported multiThrottle codes are 0' '1' '2' '3' '4' '5' only.
int WitPlusDccEx::getMultiThrottleIndex(char multiThrottle) {
    // if (logLevel>2) { console->print("WiT+DccEx:: getMultiThrottleIndex(): "); console->println(multiThrottle); }
    int mThrottle = multiThrottle - '0';


    if ((mThrottle >= 0) && (mThrottle<=5)) {
        return mThrottle;
    } else {
        return 0;
    }
}

// NO CHANGE - WiThrottle only
// the string passed in will look 'F03' (meaning turn off Function 3) or
// 'F112' (turn on function 12)
void WitPlusDccEx::processFunctionState(char multiThrottle, const String& functionData) {
    if (logLevel>1) { console->print("WiT+DccEx:: processFunctionState(): "); console->println(multiThrottle); }

    // F[0|1]nn - where nn is 0-31
    if (delegate && functionData.length() >= 3) {
        bool state = functionData[1]=='1' ? true : false;

        String funcNumStr = functionData.substring(2);
        uint8_t funcNum = funcNumStr.toInt();

        if (funcNum == 0 && funcNumStr != "0") {
            // error in parsing
        }
        else {
            if (multiThrottle == DEFAULT_MULTITHROTTLE) {
                if (delegate) delegate->receivedFunctionState(funcNum, state);
            } else {
                if (delegate) delegate->receivedFunctionStateMultiThrottle(multiThrottle,funcNum, state);
            }
        }
    }
    if (logLevel>1)  console->println("WiT+DccEx:: processFunctionState(): end");
}


// NO CHANGE - WiThrottle only
// the string passed in will look ']\[Headlight]\[Bell]\[Whistle]\[Short Whistle]\[Steam Release]\[FX5 Light]\[FX6 Light]\[Dimmer]\[Mute]\[Water Stop]\[Injectors]\[Brake Squeal]\[Coupler]\[]\[]\[]\[]\[]\[]\[]\[]\[]\[]\[]\[]\[]\[]\[]\['
void WitPlusDccEx::processRosterFunctionListEntries(char multiThrottle, const String& s) {
    if (logLevel>0) { console->print("WiT+DccEx:: processRosterFunctionListEntries(): "); console->println(multiThrottle); }

    String functions[MAX_FUNCTIONS];

    // loop
    int entries = -1;
    boolean entryFound = true;
	int entryStartPosition = 3; //ignore the first entry separator
    if (s.length() <= 3) entryFound =false;

    while ((entryFound) && (entries < MAX_FUNCTIONS)) {
	    entries++;

		// get element
		int entrySeparatorPosition = s.indexOf(ENTRY_SEPARATOR, entryStartPosition);
        if (entrySeparatorPosition == -1) entrySeparatorPosition = s.length();
		String entry = s.substring(entryStartPosition, entrySeparatorPosition);
        functions[entries] = entry;
		if (logLevel>1) { console->print("WiT+DccEx:: Function Entry: "); console->print(entries); console->print(" - "); console->println(entry); }
        
        entryStartPosition = entrySeparatorPosition + 3;
    }

    if (logLevel>0) { console->print("WiT+DccEx:: Functions for roster entry: "); console->println(entries); }

    for(int i = entries+1; i < MAX_FUNCTIONS; i++) {
        functions[i] = "";
    } 

    if (multiThrottle == DEFAULT_MULTITHROTTLE) {
        if (delegate) delegate->receivedRosterFunctionList(functions);
    } else {
        if (delegate) delegate->receivedRosterFunctionListMultiThrottle(multiThrottle, functions);
    }

    if (logLevel>1) console->println("WiT+DccEx:: processRosterFunctionListEntries(): end");
}

// NO CHANGE - WiThrottle only
void WitPlusDccEx::processSpeed(char multiThrottle, const String& speedData) {
    if (logLevel>0) { console->print("WiT+DccEx:: processSpeed(): "); console->println(multiThrottle); }
    int multiThrottleIndex = getMultiThrottleIndex(multiThrottle);

    if (delegate && speedData.length() >= 2) {
        String speedStr = speedData.substring(1);
        int speed = speedStr.toInt();

        if (speed < MIN_SPEED) {
            speed = 0;
        } else if (speed > MAX_SPEED) {
            speed = MAX_SPEED;
        }

        currentSpeed[multiThrottleIndex] = speed;
        if (multiThrottle == DEFAULT_MULTITHROTTLE) {
            currentSpeed[multiThrottleIndex] = speed;
            if (delegate) delegate->receivedSpeed(speed);
        } else {
            if (delegate) delegate->receivedSpeedMultiThrottle(multiThrottle, speed);
        }
    }

    if (logLevel>1)  console->println("WiT+DccEx:: processSpeed(): end");
}

// NO CHANGE - WiThrottle only
// note for DCC-EX there is only one system speedstep value, so all throttles will be set to the same value.
void WitPlusDccEx::processSpeedSteps(char multiThrottle, const String& speedStepData) {
    if (logLevel>0) { console->print("WiT+DccEx:: processSpeedSteps(): "); console->print(multiThrottle); console->print(" : "); console->println(speedStepData); }
    int multiThrottleIndex = getMultiThrottleIndex(multiThrottle);

    if (delegate && speedStepData.length() >= 2) {
        String speedStepStr = speedStepData.substring(1);
        int steps = speedStepStr.toInt();

        // 1 = 128step, 2 = 28step, 4 = 27step or 8 = 14step
        if (steps != 1 && steps != 2 && steps != 4 && steps != 8) {
            // error, not one of the known values
        }
        else {
            if (multiThrottle == DEFAULT_MULTITHROTTLE) {
                if (delegate) delegate->receivedSpeedSteps(steps);
                speedSteps[0] = steps;
            } else {
                if (delegate) delegate->receivedSpeedStepsMultiThrottle(multiThrottle, steps);
                speedSteps[multiThrottleIndex] = steps;
            }
        }
    }

    if (logLevel>1) console->println("WiT+DccEx:: processSpeedSteps(): end");
}

// NO CHANGE - WiThrottle only
void WitPlusDccEx::processDirection(char multiThrottle, const String& directionStr) {
    if (logLevel>0) console->println("WiT+DccEx:: processDirection()");

    int multiThrottleIndex = getMultiThrottleIndex(multiThrottle);
    if (logLevel>0) {
        console->print("WiT+DccEx:: processDirection(): throttle: "); console->println(multiThrottle);
        console->print("  DIRECTION STRING: "); console->println(directionStr);
        console->print("  LENGTH: "); console->println(directionStr.length());
    }

    // R[0|1]
    if (delegate && directionStr.length() == 2) {
        if (directionStr.charAt(1) == '0') {
            currentDirection[multiThrottleIndex] = Reverse;
        }
        else {
            currentDirection[multiThrottleIndex] = Forward;
        }

        if (multiThrottle == DEFAULT_MULTITHROTTLE) {
            if (delegate) delegate->receivedDirection(currentDirection[multiThrottleIndex]);
        } else {
            if (delegate) delegate->receivedDirectionMultiThrottle(multiThrottle, currentDirection[multiThrottleIndex]);
        }
    }

    if (logLevel>1) console->println("WiT+DccEx:: processDirection(): end"); 
}

// NO CHANGE - WiThrottle only
// should only ever be called for the non-lead locos
void WitPlusDccEx::processDirection(char multiThrottle, String& address, const String& directionStr) {
    if (logLevel>0) console->println("WiT+DccEx:: processDirection()");

    int multiThrottleIndex = getMultiThrottleIndex(multiThrottle);
    Direction direction = Forward;
    if (directionStr.charAt(1) == '0') direction = Reverse;

    if (logLevel>0) {
        console->print("WiT+DccEx:: processDirection(): (facing) throttle: "); console->println(multiThrottle);
        console->print("  Address: "); console->println(address); ; 
        console->print("  DIRECTION STRING: "); console->println(directionStr); 
        console->print(" LENGTH: "); console->println(directionStr.length());
    }

    // R[0|1]
    if (delegate && directionStr.length() == 2) {
        for(int i=0;i<locomotives[multiThrottleIndex].size();i++) {
            if (locomotives[multiThrottleIndex][i].equals(address)) {
                locomotivesFacing[multiThrottleIndex][i] = direction;

                if (multiThrottle == DEFAULT_MULTITHROTTLE) {
                    if (delegate) delegate->receivedDirection(address, direction);
                } else {
                    if (delegate) delegate->receivedDirectionMultiThrottle(multiThrottle, address, direction);
                }
                break;
            }
        }
    }

    if (logLevel>1) console->println("WiT+DccEx:: processDirection(): end"); 
}


// NO CHANGE - WiThrottle only
void WitPlusDccEx::processTrackPower(char *c, int len) {
    if (logLevel>0) console->println("WiT+DccEx:: processTrackPower()");

    if (delegate) {
        if (len > 0) {
            TrackPower state = PowerUnknown;
            if (c[0]=='0') {
                state = PowerOff;
            }
            else if (c[0]=='1') {
                state = PowerOn;
            }

            if (delegate) delegate->receivedTrackPower(state);
        }
    }
}

// NO CHANGE - WiThrottle only
void WitPlusDccEx::processAddRemove(char multiThrottle, char *c, int len) {
    if (logLevel>0) { console->print("WiT+DccEx:: processAddRemove(): "); console->println(multiThrottle); }

    if (!delegate) {
        // If no one is listening, don't do the work to parse the string
        return;
    }

    if (logLevel>0) console->printf("WiT+DccEx:: processing add/remove command %s\n", c);

    String s(c);

    bool add = (c[0] == '+');
    bool remove = (c[0] == '-');

    int p = s.indexOf(PROPERTY_SEPARATOR);
    if (p > 0) {
        String address = s.substring(1, p);
        String entry   = s.substring(p+3);

        address.trim();
        entry.trim();

        if (add) {
            if (multiThrottle == DEFAULT_MULTITHROTTLE) {
                if (delegate) delegate->addressAdded(address, entry);
            } else {
                if (delegate) delegate->addressAddedMultiThrottle(multiThrottle, address, entry);
            }
        }
        if (remove) {
            if (entry.equals("d\n") || entry.equals("r\n")) {
                if (multiThrottle == DEFAULT_MULTITHROTTLE) {
                    if (delegate) delegate->addressRemoved(address, entry);
                } else {
                    if (delegate) delegate->addressRemovedMultiThrottle(multiThrottle, address, entry);
                }
            } else {
                console->printf("WiT+DccEx:: malformed address removal: command is %s\n", entry.c_str());
                console->printf("entry length is %d\n", entry.length());
                for (int i = 0; i < entry.length(); i++) {
                    console->printf("  char at %d is %d\n", i, entry.charAt(i));
                }
            }
        }
    }

    if (logLevel>1) console->println("WiT+DccEx:: processAddRemove(): end"); 
}

// NO CHANGE - WiThrottle only
void WitPlusDccEx::processStealNeeded(char multiThrottle, char *c, int len) {
    if (logLevel>0) { console->print("WiT+DccEx:: processStealNeeded(): "); console->println(multiThrottle); }

    if (!delegate) {
        // If no one is listening, don't do the work to parse the string
        return;
    }

    if (logLevel>1) console->printf("WiT+DccEx:: processing steal needed command %s\n", c);

    String s(c);

    int p = s.indexOf(PROPERTY_SEPARATOR);
    if (p > 0) {
        String address = s.substring(0, p);
        String entry   = s.substring(p+3);

        if (multiThrottle == DEFAULT_MULTITHROTTLE) {
            if (delegate) delegate->addressStealNeeded(address, entry);
        } else {
            if (delegate) delegate->addressStealNeededMultiThrottle(multiThrottle, address, entry);
        }
    }

    if (logLevel>1) console->println("WiT+DccEx:: processStealNeeded(): end");
}

// NO CHANGE - WiThrottle only
void WitPlusDccEx::processTurnoutAction(char *c, int len) {
    if (logLevel>0) console->println("WiT+DccEx:: processTurnoutAction()");
    if (delegate) {
        String s(c);
        String systemName = s.substring(1,s.length()-1);
        TurnoutState state = TurnoutUnknown;
        if (c[0]=='2') {
            state = TurnoutClosed;
        }
        else if (c[0]=='4') {
            state = TurnoutThrown;
        }
        else if (c[0]=='1') {
            state = TurnoutUnknown;
        }
        else if (c[0]=='8') {
            state = TurnoutInconsistent;
        }

        if (delegate) delegate->receivedTurnoutAction(systemName, state);
    }
}

// NO CHANGE - WiThrottle only
void WitPlusDccEx::processRouteAction(char *c, int len) {
    if (logLevel>0) console->println("WiT+DccEx:: processRouteAction()");
    if (delegate) {
        String s(c);
        String systemName = s.substring(1,s.length()-1);
        RouteState state = RouteInconsistent;
        if (c[0]=='2') {
            state = RouteActive;
        }
        else if (c[0]=='4') {
            state = RouteInactive;
        }

        if (delegate) delegate->receivedRouteAction(systemName, state);
    }
}

// DONE
bool WitPlusDccEx::checkHeartbeat() {
    // if (logLevel>3) {
    //     console->print("WiT+DccEx:: checkHeartbeat(): heartbeatPeriod: ");
    //     console->print(heartbeatPeriod);
    //     console->print(" millis: ");
    //     console->print(millis());
    //     console->print(" heartbeatTimer: ");
    //     console->print(heartbeatTimer);
    //     console->print(" Difference: ");
    //     console->println(millis() - heartbeatTimer);
    // } 

	// if heartbeat is required and half of heartbeat period has passed, send a heartbeat and reset the timer
    if ((heartbeatPeriod > 0) && ((millis() - heartbeatTimer) > 0.5 * heartbeatPeriod * 1000)) {
    	if (logLevel>3) console->println("WiT+DccEx:: checkHeartbeat(): ");

        if (!heartbeatEnabled) {
            if (logLevel>2) console->println("WiT+DccEx:: checkHeartbeat(): heartbeat not enabled");
            heartbeatTimer = millis();
            return true;
        }

        if (isWiThrottleServer()) {
            sendDelayedCommand("*");
            setDeviceName(currentDeviceName);  // resent the device name instead of the heartbeat.  this forces the server to respond
        } else {
            sendDelayedCommand("<#>");
        }

        // // if there are any locos under control, resend all their speeds
        // if ( (timeLastLocoAcquired!=0) && ((millis() - timeLastLocoAcquired) > 5000) ) { // wait at least 5 seconds from the last time that a loco was aqcuired, to give the server time to send any existing speeds
        //     for (int i=0; i<MAX_WIT_THROTTLES; i++) {
        //         char multiThrottleChar = '0' + i;
        //         if (getNumberOfLocomotives(multiThrottleChar)>0) {
        //             setSpeed(multiThrottleChar, getSpeed(multiThrottleChar), true);
        //             int multiThrottleIndex = getMultiThrottleIndex(multiThrottleChar);
        //             if (locomotives[multiThrottleIndex].size()==1) {
        //                 setDirection(multiThrottleChar, getDirection(multiThrottleChar), true);
        //             } else {                    
        //                 for(int i=0;i<locomotives[multiThrottleIndex].size();i++) {
        //                     String loco = getLocomotiveAtPosition(multiThrottleChar,i);
        //                     setDirection(multiThrottleChar, loco, getDirection(multiThrottleChar, loco), true);
        //                 }
        //             }
        //         }
        //     }
        // }

		heartbeatTimer = millis();
		
    	if (logLevel>3) console->println("WiT+DccEx:: checkHeartbeat(): end: true");
        return true;
    }

    return false;
}

// DONE - WiThrottle only
void WitPlusDccEx::requireHeartbeat(bool needed) {
    if (isWiThrottleServer()) {
        if (needed) {
            heartbeatEnabled = true;
            sendDelayedCommand("*+");
        }
        else {
            heartbeatEnabled = false;
            sendDelayedCommand("*-");
        }
    } else { // force on
        heartbeatEnabled = true;
    }
}

// NO CHANGE
void WitPlusDccEx::processUnknownCommand(char *c, int len) {
    if (delegate && len > 0) {
        String unknownCommand = String(c);
        if (delegate) delegate->receivedUnknownCommand(unknownCommand);
    }
    heartbeatTimer = millis();
}

// ******************************************************************************************************

// DCC-EX Specific commands - outbound

// DONE
void WitPlusDccEx::dccExSetDeviceName() {
    if (logLevel>0) console->println("WiT+DccEx:: sendRequestRoster()");
     //DCC-EX - name is not relevant, so send a Command Station Status Request
    sendDelayedCommand("<s>");
    dccExSendRequestRoster();
    dccExSendRequestTurnouts();
    dccExSendRequestRoutes();
    dccExSendRequestTracks();     
}

// DONE
void WitPlusDccEx::dccExSendRequestRoster() {
    if (logLevel>0) console->println("WiT+DccEx:: sendRequestRoster()");
    sendDelayedCommand("<JR>");
}

// DONE
void WitPlusDccEx::dccExSendRequestRosterEntry(int rosterId) {
    if (logLevel>0) console->println("WiT+DccEx:: dccExSendRequestRosterEntry()");
    sendDelayedCommand("<JR " + String(rosterId) + ">");
}

// DONE
void WitPlusDccEx::dccExSendRequestTurnouts() {
    if (logLevel>0) console->println("WiT+DccEx:: dccExSendRequestTurnouts()");
    sendDelayedCommand("<JT>");
}

// DONE
void WitPlusDccEx::dccExSendRequestTurnoutEntry(int turnoutId) {
    if (logLevel>0) console->println("WiT+DccEx:: dccExSendRequestTurnoutEntry()");
    sendDelayedCommand("<JT "+String(turnoutId)+">");
}

// DONE
void WitPlusDccEx::dccExSendRequestRoutes() {
    if (logLevel>0) console->println("WiT+DccEx:: dccExSendRequestRoutes()");
    sendDelayedCommand("<JA>");
}

// DONE
void WitPlusDccEx::dccExSendRequestRouteEntry(int routeId) {
    if (logLevel>0) console->println("WiT+DccEx:: dccExSendRequestRouteEntry()");
    sendDelayedCommand("<JA "+String(routeId)+">");
}

// DONE
void WitPlusDccEx::dccExSendRequestTracks() {
    if (logLevel>0) console->println("WiT+DccEx:: dccExSendRequestTracks()");
    sendDelayedCommand("<=>");
}

// TODO - DELAY 
void WitPlusDccEx::dccExSetCurrentFastTime(const String& s) {
    if (logLevel>0) console->println("WiT+DccEx:: dccExSetCurrentFastTime()");
}

// DONE
bool WitPlusDccEx::dccExSendAddLocomotive(char multiThrottle, String address) {
    if (logLevel>0) { console->print("WiT+DccEx:: dccExSendAddLocomotive(): "); console->print(multiThrottle); console->print(" : "); console->println(address); }

    int multiThrottleIndex = getMultiThrottleIndex(multiThrottle);
    bool ok = false;
    if (address[0] == 'S' || address[0] == 'L') {
        String dccAddress = address.substring(1);
        sendDelayedCommand("<t " + dccAddress + ">");
        addLocoToThrottle(multiThrottle, address);
        ok = true;

        // if it is in the roster, send the functions
        String functions[MAX_FUNCTIONS];
        for (int i=0; i<MAX_WIT_THROTTLES; i++) {
            int position = getLocoIndexInThrottle('0'+i, address);
            if (position==0) { // lead loco only
                int rosterIndex = getLocoIndexInRoster(address);
                for (int j=0; j<MAX_FUNCTIONS; j++) {
                    functions[j] = rosterFunctionLabels[j][rosterIndex];
                    LocomotivesFunctionIsLatching[multiThrottleIndex][j] = rosterFunctionIsLatching[j][rosterIndex];
                    LocomotivesFunctionStates[multiThrottleIndex] = false;
                    if (logLevel>2) { console->print("WiT+DccEx:: dccExSendAddLocomotive() function: "); console->println(functions[j]); }
                }

                if (multiThrottle == DEFAULT_MULTITHROTTLE) {
                    if (delegate) delegate->receivedRosterFunctionList(functions);
                } else {
                    if (delegate) delegate->receivedRosterFunctionListMultiThrottle(multiThrottle, functions);
                }
            }
        }
    }

    if (logLevel>1) { console->print("WiT+DccEx:: dccExSendAddLocomotive(): end : ");  console->println(ok); }
    return ok;
}

// DONE
bool WitPlusDccEx::dccExReleaseLocomotive(char multiThrottle, String address) {
    if (logLevel>0) { console->print("WiT+DccEx:: dccExReleaseLocomotive(): "); console->print(multiThrottle); console->print(" : "); console->println(address); }

    // int multiThrottleIndex = getMultiThrottleIndex(multiThrottle);
    bool ok = false;
    if (address[0] == 'S' || address[0] == 'L') {
        String dccAddress = address.substring(1);
        removeLocoFromThrottle(multiThrottle, address);
        ok = true;
    }
    if (logLevel>1) { console->print("WiT+DccEx:: dccExReleaseLocomotive(): end : ");  console->println(ok); }
    return ok;
}

// DONE
void WitPlusDccEx::dccExSetFunction(char multiThrottle, int funcnum, bool pressed) {
    dccExSetFunction(multiThrottle, ALL_LOCOS_ON_THROTTLE, funcnum, pressed);
}
// TODO - not covering latching yet
void WitPlusDccEx::dccExSetFunction(char multiThrottle, String address, int funcnum, bool pressed) {
    if (logLevel>2) { console->print("WiT+DccEx:: dccExSetFunction(): throttle: "); console->print(multiThrottle); console->print(" : "); console->print(address); console->print(" : "); console->print(funcnum); console->print(" : "); console->print(pressed ? "pressed" : "released"); console->print(" : "); console->println(funcnum); }

    int multiThrottleIndex = getMultiThrottleIndex(multiThrottle);

    bool isLatching = true;
    if (funcnum==2) isLatching = false;

    if (!address.equals(ALL_LOCOS_ON_THROTTLE)) {
        isLatching = LocomotivesFunctionIsLatching[multiThrottleIndex];
    }
    if (logLevel>2) { console->print("WiT+DccEx:: dccExSetFunction(): isLatching: "); console->println((isLatching ? "1" : "0")); }

    if (address.equals(ALL_LOCOS_ON_THROTTLE))  {  // all on throttle
        for (int i=0;i<locomotives[getMultiThrottleIndex(multiThrottle)].size();i++) {
            String dccAddress = locomotives[multiThrottleIndex][i].substring(1);
            if (!isLatching) {
                sendDelayedCommand("<F " + dccAddress + " " + String(funcnum) + " " + (pressed ? "1" : "0") + ">");
            } else {
                if (!pressed) { // just release, ignore press
                    bool newState = !LocomotivesFunctionStates[multiThrottleIndex];
                    sendDelayedCommand("<F " + dccAddress + " " + String(funcnum) + " " + (newState ? "1" : "0") + ">");
                    LocomotivesFunctionStates[multiThrottleIndex] = newState;
                }
            }
        }
        return;

    } else {
    
        int index = getLocoIndexInThrottle(multiThrottle, address);

        String oneAddress = address;
        if (address.length()==0) { // blank. get the first loco on the throttle
            oneAddress = locomotives[multiThrottleIndex][0];
            index = 0;
        }

        if (oneAddress.length()==0) return;

        if (logLevel>2) { console->print("WiT+DccEx:: dccExSetFunction(): multiThrottle: "); console->print(multiThrottle); console->print(" address: "); console->print(address); console->print(" index: "); console->println(index); }

        String dccAddress = oneAddress.substring(1);
        if (!isLatching) {
            sendDelayedCommand("<F " + dccAddress + " " + String(funcnum) + " " + String(pressed ? 1 : 0) + ">");
        } else {
            if (!pressed) { // just release, ignore press
                bool newState = !LocomotivesFunctionStates[multiThrottleIndex];
                sendDelayedCommand("<F " + dccAddress + " " + String(funcnum) + " " + (newState ? "1" : "0") + ">");
                LocomotivesFunctionStates[multiThrottleIndex] = newState;
            }
        }
    }
}

// DONE
int WitPlusDccEx::dccExGetSpeedSteps() {
    return dccExGetSpeedSteps(DEFAULT_MULTITHROTTLE);
}

// DONE
int WitPlusDccEx::dccExGetSpeedSteps(char multiThrottle) {
    if (logLevel>0) { console->print("WiT+DccEx:: dccExGetSpeedSteps(): "); console->println(multiThrottle); }

    int multiThrottleIndex = getMultiThrottleIndex(multiThrottle);
    return speedSteps[multiThrottleIndex];
}

//DONE
bool WitPlusDccEx::dccExSetSpeedSteps(int steps) {
    return dccExSetSpeedSteps(DEFAULT_MULTITHROTTLE, steps);
}

// DONE 
bool WitPlusDccEx::dccExSetSpeedSteps(char multiThrottle, int steps) {
    return true;

    if (logLevel>0) { console->print("WiT+DccEx:: dccExSetSpeedSteps(): "); console->print(multiThrottle); console->print(" : "); console->println(steps); }

    // multithrottles is ignored, 
    // as DCC-EX does not support different speed steps for different throttles

    // 1 = 128step, 2 = 28step, 4 = 27step or 8 = 14step
    if (steps==1) { sendDelayedCommand("<D SPEED28>"); }
    else if (steps==2) { sendDelayedCommand("<D SPEED128>"); }
    else { // DCC-EX does not support 27 or 14 steps
        console->print("WiT+DccEx:: setSpeedSteps(): Error, not one of the known values");
        return false;
    }

    for (int multiThrottleIndex=0; multiThrottleIndex<MAX_WIT_THROTTLES; multiThrottleIndex++)
        speedSteps[multiThrottleIndex] = steps;

    return true;
}

// IN PROGRESS - check consists work, otherwise ok
bool WitPlusDccEx::dccExSetSpeed(char multiThrottle, int speed) {
    if (logLevel>0) { console->print("WiT+DccEx:: dccExSetSpeed(): "); console->print(multiThrottle); console->print(" : "); console->println(speed); }

    int multiThrottleIndex = getMultiThrottleIndex(multiThrottle);

    if (speed < -1 || speed > 126) {
        return false;
    }
    if (!locomotiveSelected[multiThrottleIndex]) {
        return false;
    }

    if (speed != currentSpeed[multiThrottleIndex]) {
        for (int i=0;i<locomotives[multiThrottleIndex].size();i++) {
            String dccExAddress = locomotives[multiThrottleIndex][i].substring(1);
            bool direction = locomotivesFacing[multiThrottleIndex][i];
            if (getDirection(multiThrottle) != direction) {
                direction = !direction;
            }
            String cmd = "<t " + dccExAddress + " " + String(speed) + (direction ? " 1" : " 0") + ">";
            sendDelayedCommand(cmd);
        }
        currentSpeed[multiThrottleIndex] = speed;

        lastSpeedCommandSent[multiThrottleIndex] = millis();
    }
    return true;
}

// IN PROGRESS - check consists work, otherwise ok
bool WitPlusDccEx::dccExSetDirection(char multiThrottle, Direction direction) {
    return dccExSetDirection(multiThrottle, "*", direction, false);
}
bool WitPlusDccEx::dccExSetDirection(char multiThrottle, String address, Direction direction) {
    return dccExSetDirection(multiThrottle, address, direction, false);
}
// DONE - UNSURE IF THIS WILL WORK - need to look at forceSend
bool WitPlusDccEx::dccExSetDirection(char multiThrottle, String address, Direction direction, boolean forceSend) {
    if (logLevel>0) { console->print("WiT+DccEx:: dccExSetDirection(): address: "); console->print(address); console->print(" throttle: "); 
    console->print(multiThrottle); console->print(" direction: "); console->print(direction); console->print(" forceSend: "); console->println(forceSend); }

    int multiThrottleIndex = getMultiThrottleIndex(multiThrottle);
    if (!locomotiveSelected[multiThrottleIndex]) {
        return false;
    }

    String directionString = (direction == Reverse) ? "0" : "1";
    Direction currentDir = currentDirection[multiThrottleIndex];
    int locoIndex = -1;
    if (!address.equals(ALL_LOCOS_ON_THROTTLE)) {
        for(int i=0;i<locomotives[multiThrottleIndex].size();i++) {
            if (locomotives[multiThrottleIndex][i].equals(address)) {
                locoIndex = i;
                currentDir = locomotivesFacing[multiThrottleIndex][i];
                break;
            }
        }
    }

    if (direction != currentDir) {
        int speed = currentSpeed[multiThrottleIndex];
        for (int i=0;i<locomotives[multiThrottleIndex].size();i++) {
            String dccExAddress = locomotives[multiThrottleIndex][i].substring(1);
            bool facing = locomotivesFacing[multiThrottleIndex][i];
            bool locoDirection = direction;
            if (facing == direction) {
                locoDirection = !locoDirection;
            }
            String cmd = "<t " + dccExAddress + " " + String(speed) + (locoDirection ? " 1" : " 0") + ">";
            sendDelayedCommand(cmd);
        }

        if (locoIndex == -1) { // all locos
            currentDirection[multiThrottleIndex] = direction;
        } else {
            locomotivesFacing[multiThrottleIndex][locoIndex] = direction;
        }
    }
    return true;
}

// DONE
void WitPlusDccEx::dccExEmergencyStop() {
    dccExEmergencyStop('*', ALL_LOCOS_ON_THROTTLE);
}    
// DONE
void WitPlusDccEx::dccExEmergencyStop(char multiThrottle) {
    dccExEmergencyStop(multiThrottle, ALL_LOCOS_ON_THROTTLE);
}
// DONE
// address is currently ignored
void WitPlusDccEx::dccExEmergencyStop(char multiThrottle, String address) {
    if (logLevel>0) { console->print("WiT+DccEx:: dccExEmergencyStop(): throttle: "); 
    console->print(multiThrottle); console->print(" address: "); console->println(address); }

    if (multiThrottle=='*') { // all throttles,]. all locos
        for (int i=0; i<MAX_WIT_THROTTLES; i++) {
            dccExSetSpeed('0'+i, -1);
        }
    } else { // one throttle
        dccExSetSpeed(multiThrottle, -1);
    }
}

// DONE
// Turn on/off all tracks
void WitPlusDccEx::dccExSetTrackPower(TrackPower state) {
    if (logLevel>2) console->println("WiT+DccEx:: dccExSetTrackPower()");
    sendDelayedCommand("<" + String((state==PowerOn) ? "1" : "0") + ">");
}

// DONE
// Turn on/off a specific track letter
void WitPlusDccEx::dccExSetTrackPower(TrackPower state, char track) {
    if (logLevel>2) console->println("WiT+DccEx:: dccExSetTrackPower()");
    sendDelayedCommand("<" + String((state==PowerOn) ? "1" : "0") + " " + String(track) + ">");
}

// DONE
// Turn on/off a specific type of track
void WitPlusDccEx::dccExSetTrackPower(TrackPower state, String track) {
    if (logLevel>2) console->println("WiT+DccEx:: dccExSetTrackPower()");
    sendDelayedCommand("<" + String((state==PowerOn) ? "1" : "0") + " " + track + ">");
}

// DONE
// Open/Close a Turnout/Point
bool WitPlusDccEx::dccExSetTurnout(String address, TurnoutAction action) {
    if (logLevel>2) console->println("WiT+DccEx:: dccExSetTurnout()");

    TurnoutState currentState = TurnoutClosed;
    String newStateString = "T";

    int turnoutId = address.toInt();
    int turnoutIndex = getTurnoutIndexInTurnoutsList(turnoutId);

    if (turnoutIndex>=0) {
        currentState = turnoutsListStates[turnoutIndex];
    } // if we don't have it in our list, assume it is currently closed

    switch (action) {
        case TurnoutClose: {
            newStateString = "C";
            break;
        }
        case TurnoutToggle: {
            if (currentState==TurnoutClosed) {
                newStateString = "T";
            } else { 
                newStateString = "C";
            }
            break;
        }
        case TurnoutThrow:
        default: {
            // newStateString = "T"; // already the default
            break;
        }
    }

    // Note: Don't set the state in the internal array here,
    // as the new state will be sent back from the server if it worked.

    String cmd = "<T " + address + " " + newStateString +">";
    sendDelayedCommand(cmd);

    return true;
}

// DONE
bool WitPlusDccEx::dccExSetRoute(String address) {
    String cmd = "</START " + address +">";
    sendDelayedCommand(cmd);
    return true;
}

// DCC-EX Specific commands - inbound

// DONE
void WitPlusDccEx::dccExProcessCommandStationInfo(char *c, int len) {
    if (logLevel>0) console->println("WiT+DccEx:: dccExProcessCommandStationInfo()");

    if (delegate) {
        
        String s(c);
        int versionEnd = 0;
        for (int i=10;i<len;i++) {
            if (c[i] == '/') {
                versionEnd = i - 1;
                break;
            }
        }
        if (versionEnd > 0) {
            String protocolVersion = s.substring(10, versionEnd);
            if (delegate) delegate->receivedVersion(protocolVersion);
        }

        // String serverDescription = s.substring(versionEnd + 3, len);
        // if (delegate) delegate->receivedServerDescription(serverDescription);
        if (delegate) delegate->receivedServerDescription(s.substring(versionEnd + 3, len));
        if (delegate) delegate->receivedServerType("DCC-EX");
    }
}

// DONE
void WitPlusDccEx::dccExProcessEmergencyStop(char *c, int len) {
    if (logLevel>0) console->println("WiT+DccEx:: dccExProcessEmergencyStop()");

    for(int multiThrottleIndex = 0; multiThrottleIndex < MAX_WIT_THROTTLES; multiThrottleIndex++) {
        if (delegate) delegate->receivedSpeedMultiThrottle(multiThrottleIndex, 0);
        // for(int i=0;i<locomotives[multiThrottleIndex].size();i++) {
        // }
    }
}

// IN PROGRESS - check consists work, otherwise ok
void WitPlusDccEx::dccExProcessLocos(char *c, int len) {
    if (logLevel>0) { console->print("WiT+DccEx:: dccExProcessLocos() c:"); console->println(c); }
    
    int start = 1; // space after command
    String dccAddress = getParameter(c, start, len, 1);
    String loco = getLocoStringFromDccAddress(dccAddress);
    String speedByte = getParameter(c, start, len, 3);
    String functMap = getParameter(c, start, len, 4);

    // if (logLevel>2) { 
    //     console->print("WiT+DccEx:: dccExProcessLocos()");
    //     console->print(" loco: «");
    //     console->print(loco);
    //     console->print("» speedByte: «");
    //     console->print(speedByte);
    //     console->print("» functMap: «");
    //     console->print(functMap);
    //     console->println("»");
    // }

    for (int multiThrottleIndex=0; multiThrottleIndex<MAX_WIT_THROTTLES; multiThrottleIndex++) {
        char multiThrottle = '0'+multiThrottleIndex; 
        if (getLocoIndexInThrottle(multiThrottle, loco) >= 0) {
            // ignore the incoming command if we sent a speed command recently
            double timeDifference = millis() - lastSpeedCommandSent[multiThrottleIndex];
            console->print("WiT+DccEx:: dccExProcessLocos(): timeDifference: "); console->println(timeDifference);
            if (timeDifference > 1000) {
                int speed = getSpeedFromSpeedByte(speedByte);
                    if (logLevel>2) { console->print("WiT+DccEx:: dccExProcessLocos() speed: "); console->println(speed); }
                if ( (speed >= 0) && (speed<127) ) {
                    if (delegate) delegate-> receivedSpeedMultiThrottle(multiThrottle, speed);
                }
                Direction dir = getDirectionFromSpeedByte(speedByte);
                if (delegate) delegate->receivedDirectionMultiThrottle(multiThrottle, dir);
            }
            if ( (getLocoIndexInThrottle(multiThrottle, loco) == 0) && (functMap.length()>0) ) { // lead loco
                int functMapValue = functMap.toInt();
                boolean bit;
                for (int funcNum=0; funcNum<MAX_FUNCTIONS; funcNum++) {
                    bit = ((functMapValue >> funcNum) & 1) ? true : false;

                    if (bit != LocomotivesFunctionStates[funcNum]) {
                        if (logLevel>2) { console->print("WiT+DccEx:: dccExProcessLocos(): funcNum: "); console->print(funcNum); console->print(" bit: "); console->println((bit ? "true" : "false")); }

                        if (delegate) delegate->receivedFunctionStateMultiThrottle(multiThrottle, funcNum, bit);                         
                        LocomotivesFunctionStates[funcNum] = bit;
                    }
                }

            }
        }
    }
}

// DONE  
// <p0|1>  
// <p0|1 A..H|MAIN|PROG|DC|DCX>
// ignoring <p 0|1 A..H|MAIN|PROG|DC|DCX>   (p space 0|1 ..)
void WitPlusDccEx::dccExProcessPower(char *c, int len) {
    if (logLevel>0) { console->print("WiT+DccEx:: dccExProcessPower() c: "); console->println(c); }

    String s(c);
    // if (logLevel>0) { console->print("WiT+DccEx:: dccExProcessPower() s: "); console->println(s); }

    bool isOn = s.charAt(1)=='1' ? true : false;
    TrackPower state = isOn ? PowerOn : PowerOff;

    // if (logLevel>0) { console->print("WiT+DccEx:: dccExProcessPower() state: "); console->println(state); }

    int start = 1; // space after command and state
    String trackString = getParameter(c, start, len, 1);

    if (trackString.length()==0) { // global power
        for (int i=0; i<MAX_TRACKS; i++) {
            trackPower[i] = state;
        }
        if (delegate) delegate->receivedTrackPower(state);

    } else if ( (trackString.length()==1) && (trackString.charAt(0)>='A') && (trackString.charAt(0)<='H')) { // track only
        trackPower[trackString.charAt(0)-'A'] = state;
    
    } else { // track type  - MAIN|PROG|DC|DCX
        TrackType type = getTrackType(trackString);
        
        // if (logLevel>0) { console->print("WiT+DccEx:: dccExProcessPower() trackString: "); console->println(trackString); }
        // if (logLevel>0) { console->print("WiT+DccEx:: dccExProcessPower() type: "); console->println(type); }

        for (int i=0; i<MAX_TRACKS; i++) {
            if (trackType[i]==type) trackPower[i] = state;
        }
    }

    // now count how many there are and how many of those are on
    int noTracks = 0;
    int noTracksWithPowerOn = 0;
    for (int i=0; i<MAX_TRACKS;i++) {
        // if (logLevel>0) { console->print("WiT+DccEx:: dccExProcessPower() i: "); console->print(i); console->print(" type: "); console->print(trackType[i]); console->print(" power: "); console->println(trackPower[i]); }

        if ( (trackType[i]==TRACK_TYPE_MAIN) || (trackType[i]==TRACK_TYPE_MAIN_INV) 
        || (trackType[i]==TRACK_TYPE_DC) || (trackType[i]==TRACK_TYPE_DCX) ) {
            noTracks++;
            if (trackPower[i]==PowerOn) noTracksWithPowerOn++;
        }
    }

    // if (logLevel>0) { console->print("WiT+DccEx:: dccExProcessPower() noTracksWithPowerOn: "); console->print(noTracksWithPowerOn); console->print(" noTracks: "); console->println(noTracks); }

    if (noTracksWithPowerOn >= (noTracks*.75)) {
        if (delegate) delegate->receivedTrackPower(PowerOn); 
    } else {
        if (delegate) delegate->receivedTrackPower(PowerOff); 
    }
}

// DONE
TrackType WitPlusDccEx::getTrackType(String typeString) {
    if (typeString.equals("MAIN")) return TRACK_TYPE_MAIN;
    else if (typeString.equals("MAIN_INV")) return TRACK_TYPE_MAIN_INV;
    else if (typeString.equals("PROG")) return TRACK_TYPE_PROG;
    else if (typeString.equals("DC")) return TRACK_TYPE_DC;
    else if (typeString.equals("DCX")) return TRACK_TYPE_DCX;
    else if (typeString.equals("AUTO")) return TRACK_TYPE_AUTO;
    else if (typeString.equals("EXT")) return TRACK_TYPE_EXT;
    else if (typeString.equals("BOOST")) return TRACK_TYPE_BOOST;
    return TRACK_TYPE_NONE;
}

// DONE
// <jT>
// <jT id1 id2 id3 ...>
// <jT id X>
// <jT id state ["desc"]>
void WitPlusDccEx::dccExProcessTurnouts(char *c, int len) {
    if (logLevel>0) console->println("WiT+DccEx:: dccExProcessTurnouts()");

    String s(c);
    int start = 1; // space after command
    turnoutsListCounter = 0;
    int turnoutsCount = getNumberOfParameters(c, start, len);

    if (turnoutsCount == 0) { // no defined turnouts/points <jT>
        turnoutsListNumberOfEntries = 0;
        clearDccExTurnouts();
        if (delegate) delegate->receivedTurnoutEntries(0);

    } else { // turnouts list  <jT id1 id2 id3 ...>
        bool isIndividualTurnout = false;
        String turnoutIdString = getParameter(c,start,len, 0);
        String turnoutState = getParameter(c,start,len, 1);
        if (logLevel>0) { console->print("WiT+DccEx:: dccExProcessTurnouts() turnoutId: "); console->print(turnoutIdString); console->print(" state: "); console->println(turnoutState); }

        if ( (turnoutsCount==3) || (turnoutsCount== 4) ) { // possible individual
            if (turnoutIdString.equals("X")) return; // unknown id
            if ( (turnoutState.equals("C")) || (turnoutState.equals("T")) ) {
                isIndividualTurnout = true;
            }
        }
        if (!isIndividualTurnout) { // list
            if (logLevel>0) { console->println("WiT+DccEx:: dccExProcessTurnouts() processing list"); }
            
            if (!turnoutsListReceived) {
                clearDccExTurnouts();
                turnoutsListNumberOfEntries = turnoutsCount;
                if (delegate) delegate->receivedTurnoutEntries(turnoutsCount);
                for (int i=0; i<turnoutsCount; i++) {
                    int turnoutId = getParameter(c, start, len, i).toInt();
                    turnoutsListIds.push_back(turnoutId);
                    turnoutsListNames.push_back("");
                    turnoutsListEntriesReceived.push_back(false);
                    turnoutsListStates.push_back(TurnoutInconsistent);

                    dccExSendRequestTurnoutEntry(turnoutId);
                    if (logLevel>0) { console->print("WiT+DccEx:: dccExProcessTurnouts() id: "); console->println(turnoutId); }
                }
                turnoutsListIndex = -1;

            } // else ignore it if we already have it

        } else { // <jt id state ["desc"]>  individual turnout/point
            if (logLevel>0) console->println("WiT+DccEx:: dccExProcessTurnouts() Individual turnout/point");
            String turnoutName = getParameter(c,start,len, 2);
            dccExProcessTurnoutEntry(turnoutIdString.toInt(), turnoutName, turnoutState.equals("C") ? TurnoutClosed : TurnoutThrown);
        }
    }
}

// DONE
void WitPlusDccEx::dccExProcessTurnoutEntry(int turnoutId, String turnoutName, TurnoutState turnoutState) {
    if (logLevel>0) console->println("WiT+DccEx:: dccExProcessTurnoutEntry()");

    turnoutsListIndex = getTurnoutIndexInTurnoutsList(turnoutId);
    if (turnoutsListIndex<0) return; // not in the list

    if (logLevel>0) { console->print("WiT+DccEx:: dccExProcessTurnoutEntry(): turnoutsListIndex:"); console->println(turnoutsListIndex); }

    turnoutsListNames[turnoutsListIndex] = turnoutName.substring(1,turnoutName.length()-2);
    turnoutsListStates[turnoutsListIndex] = turnoutState;
    turnoutsListEntriesReceived[turnoutsListIndex] = true; 

    turnoutsListCounter++;
    if (delegate) {
        bool fullyReceived = true;
        if (logLevel>0) { console->println("WiT+DccEx:: dccExProcessTurnoutEntry(): check start");}
        for (int i=0; i<turnoutsListNumberOfEntries; i++) {
            if (!turnoutsListEntriesReceived[i]) {
                fullyReceived = false;
                break;
            }
        }

        if (logLevel>0) { console->println("WiT+DccEx:: dccExProcessTurnoutEntry(): check end");}

        if (fullyReceived) {  // have them all now
            if (logLevel>0) console->println("WiT+DccEx:: dccExProcessTurnoutEntry(): all Turnout/Point entries received");
            for (int i=0; i<turnoutsListNumberOfEntries; i++) {
                if (delegate) delegate->receivedTurnoutEntry(i, String(turnoutsListIds[i]), turnoutsListNames[i], turnoutsListIds[i]);
            }
        }
    }
    if (logLevel>0) { console->println("WiT+DccEx:: dccExProcessTurnoutEntry(): end");}

}

// DONE
void WitPlusDccEx::clearDccExTurnouts() {
    // if (logLevel>0) console->println("WiT+DccEx:: clearDccExTurnouts()");

    turnoutsListIds.clear();
    turnoutsListNames.clear();
    turnoutsListStates.clear();
    turnoutsListEntriesReceived.clear();
    turnoutsListNumberOfEntries = 0;
}

// DONE
int WitPlusDccEx::getTurnoutIndexInTurnoutsList(int turnoutId) {
    for(int i=0;i<turnoutsListNumberOfEntries;i++) {
        // if (logLevel>0) { console->print("WiT+DccEx:: getTurnoutIndexInTurnoutsLst(): turnoutsListIds[i]:"); console->println(turnoutsListIds[i]); }
    
        if (turnoutsListIds[i] == turnoutId) {
            return i;
        }
    } 
    return -1;
}

// DONE
// <H id state>
void WitPlusDccEx::dccExProcessTurnoutUpdate(char *c, int len) {
    if (logLevel>0) console->println("WiT+DccEx:: dccExProcessTurnoutUpdate()");

    String s(c);
    int start = 1; // space after command
    String idString = getParameter(c,start,len, 0);
    // String newStateString = getParameter(c,start,len, 1);

    int turnoutId = idString.toInt();
    TurnoutState newTurnoutState = getParameter(c,start,len, 1).equals("C") ? TurnoutClosed : TurnoutThrown;

    int turnoutIndex = getTurnoutIndexInTurnoutsList(turnoutId);

    if (turnoutIndex<0) { // not in the list, so add it
        turnoutsListIds.push_back(turnoutId);
        turnoutsListNames.push_back("-" + String(turnoutId)+"-");
        turnoutIndex = getTurnoutIndexInTurnoutsList(turnoutId);
    }

    if (turnoutIndex>=0) { // we have it in the list
        turnoutsListStates[turnoutIndex] = newTurnoutState;
    }
    if (delegate) delegate->receivedTurnoutAction(idString, newTurnoutState);
}

// TODO
// <jA>
// <jA id0 id1 id2 ..>
// <jA id X>
// <jA id type ["desc"]>
void WitPlusDccEx::dccExProcessRoutes(char *c, int len) {
    if (logLevel>0) console->println("WiT+DccEx:: dccExProcessRoutes()");

    String s(c);
    int start = 1; // space after command
    routesListCounter = 0;
    int routesCount = getNumberOfParameters(c, start, len);

    if (routesCount == 0) { // no defined routes/automations <jA>
        routesListNumberOfEntries = 0;
        clearDccExRoutes();
        if (delegate) delegate->receivedRouteEntries(0);

    } else { // routes list  <jA id1 id2 id3 ...>
        bool isIndividualRoute = false;
        String routeIdString = getParameter(c,start,len, 0);
        String routeType = getParameter(c,start,len, 1);
        if (logLevel>0) { console->print("WiT+DccEx:: dccExProcessRoutes() routeId: "); console->print(routeIdString); console->print(" type: "); console->println(routeType); }

        if ( (routesCount==3) || (routesCount== 4) ) { // possible individual
            if (routeIdString.equals("X")) return; // unknown id
            if ( (routeType.equals("R")) || (routeType.equals("A")) ) {
                isIndividualRoute = true;
            }
        }
        if (!isIndividualRoute) { // list
            if (logLevel>0) { console->println("WiT+DccEx:: dccExProcessRoutes() processing list"); }
            
            if (!routesListReceived) {
                clearDccExRoutes();
                routesListNumberOfEntries = routesCount;
                if (delegate) delegate->receivedRouteEntries(routesCount);
                for (int i=0; i<routesCount; i++) {
                    int routeId = getParameter(c, start, len, i).toInt();
                    routesListIds.push_back(routeId);
                    routesListNames.push_back("");
                    routesListLabels.push_back("Set");
                    routesListEntriesReceived.push_back(false);
                    routesListTypes.push_back(RouteTypeRoute);

                    dccExSendRequestRouteEntry(routeId);
                    if (logLevel>0) { console->print("WiT+DccEx:: dccExProcessRoutes() id: "); console->println(routeId); }
                }
                routesListIndex = -1;

            } // else ignore it if we already have it

        } else { // <ja id type ["desc"]>  individual route/automation
            if (logLevel>0) console->println("WiT+DccEx:: dccExProcessRoutes() Individual route");
            String routeName = getParameter(c,start,len, 2);
            dccExProcessRouteEntry(routeIdString.toInt(), routeName, routeType.equals("R") ? RouteTypeRoute : RouteTypeAutomation);
        }
    }    
}

// DONE
void WitPlusDccEx::dccExProcessRouteEntry(int routeId, String routeName, RouteType routeType) {
    if (logLevel>0) console->println("WiT+DccEx:: dccExProcessRouteEntry()");

    routesListIndex = getRouteIndexInRoutesList(routeId);
    if (routesListIndex<0) return; // not in the list

    if (logLevel>0) { console->print("WiT+DccEx:: dccExProcessRouteEntry(): routesListIndex:"); console->println(routesListIndex); }

    routesListNames[routesListIndex] = routeName.substring(1,routeName.length()-2);
    routesListTypes[routesListIndex] = routeType;
    routesListEntriesReceived[routesListIndex] = true; 

    routesListCounter++;
    if (delegate) {
        bool fullyReceived = true;
        if (logLevel>0) { console->println("WiT+DccEx:: dccExProcessRouteEntry(): check start");}
        for (int i=0; i<routesListNumberOfEntries; i++) {
            if (!routesListEntriesReceived[i]) {
                fullyReceived = false;
                break;
            }
        }

        if (logLevel>0) { console->println("WiT+DccEx:: dccExProcessRouteEntry(): check end");}

        if (fullyReceived) {  // have them all now
            if (logLevel>0) console->println("WiT+DccEx:: dccExProcessRouteEntry(): all Route entries received");
            for (int i=0; i<routesListNumberOfEntries; i++) {
                if (delegate) delegate->receivedRouteEntry(i, String(routesListIds[i]), routesListNames[i], routesListIds[i]);
            }
        }
    }
    if (logLevel>0) { console->println("WiT+DccEx:: dccExProcessRouteEntry(): end");}
}

// DONE
void WitPlusDccEx::clearDccExRoutes() {
    // if (logLevel>0) console->println("WiT+DccEx:: clearDccExRoutes()");

    routesListIds.clear();
    routesListNames.clear();
    routesListLabels.clear();
    routesListTypes.clear();
    routesListEntriesReceived.clear();
    turnoutsListNumberOfEntries = 0;
}

// DONE
int WitPlusDccEx::getRouteIndexInRoutesList(int routeId) {
    for(int i=0;i<routesListNumberOfEntries;i++) {
        // if (logLevel>0) { console->print("WiT+DccEx:: getRouteIndexInRoutesLst(): routesListIds[i]:"); console->println(routesListIds[i]); }
    
        if (routesListIds[i] == routeId) {
            return i;
        }
    } 
    return -1;
}

// TODO
void WitPlusDccEx::dccExProcessRouteUpdate(char *c, int len) {
    if (logLevel>0) console->println("WiT+DccEx:: dccExProcessRouteUpdate()");
}

// DONE 
void WitPlusDccEx::dccExProcessRoster(char *c, int len) {
    if (logLevel>0) console->println("WiT+DccEx:: dccExProcessRoster()");

    int start = 1; // space after command
    rosterCounter = 0;
    int rosterCount = getNumberOfParameters(c, start, len);

    if (rosterCount==0) {  // no roster entries

        rosterNumberOfEntries = 0;
        clearDccExRoster();
        if (delegate) delegate->receivedRosterEntries(0);
    } else {
        if ( (rosterCount < 3) || (getParameter(c, start, len, 1).charAt(0) != '"') ) { // loco list
            if (logLevel>0) console->println("WiT+DccEx:: dccExProcessRoster() Roster List");

            if (!rosterListReceived) {
                clearDccExRoster();
                rosterNumberOfEntries = rosterCount;
                if (delegate) delegate->receivedRosterEntries(rosterCount);
                for (int i=0; i<rosterCount; i++) {
                    int rosterId = getParameter(c, start, len, i).toInt();
                    rosterIds.push_back(rosterId);
                    rosterAddresses.push_back(((rosterId<=127) ? "S" : "L") + String(rosterId));
                    rosterNames.push_back("");
                    for (int j=0; j<MAX_FUNCTIONS; j++) {
                        rosterFunctionLabels[j].push_back("");
                        rosterFunctionIsLatching[j].push_back("");
                    }
                    rosterEntriesReceived.push_back(false);

                    dccExSendRequestRosterEntry(rosterId);
                    if (logLevel>0) { console->print("WiT+DccEx:: dccExProcessRoster() Roster: "); console->println(rosterId); }
                }
                rosterIndex = -1;
            }

        } else { // individual loco
            if (logLevel>0) console->println("WiT+DccEx:: dccExProcessRoster() Individual Loco");
            int rosterId = getParameter(c, start, len, 0).toInt();
            String rosterName = getParameter(c, start, len, 1);
            String functionList = getParameter(c, start, len, 2);
            dccExProcessRosterEntry(rosterId, rosterName, functionList);
        }
    }
}

// DONE
void WitPlusDccEx::dccExProcessRosterEntry(int rosterId, String rosterName, String functionList) {
    if (logLevel>0) { console->print("WiT+DccEx:: dccExProcessRosterEntry(): rosterName:"); console->println(rosterName); }
    
    rosterIndex = getLocoIndexInRoster(rosterId);
    if (rosterIndex<0) return;

    if (logLevel>0) { console->print("WiT+DccEx:: dccExProcessRosterEntry(): rosterIndex:"); console->println(rosterIndex); }

    rosterNames[rosterIndex] = rosterName.substring(1,rosterName.length()-2);
    rosterAddresses[rosterIndex] = ((rosterId<=127) ? "S" : "L") + String(rosterId);
    if (functionList.length()>2) { // exclude the opening and closing quotes
        
        int bufferSize = functionList.length() + 1;
        char functionChars[bufferSize];
        functionList.toCharArray(functionChars, bufferSize);

        int numberOfFunctions = getNumberOfParameters(functionChars, 1, functionList.length(), "/");
        for (int i=0; (i<MAX_FUNCTIONS) && (i<numberOfFunctions); i++) {
            rosterFunctionLabels[i][rosterIndex] = getParameter(functionChars, 1, functionList.length(), "/", i);
            rosterFunctionIsLatching[i][rosterIndex] = rosterFunctionLabels[i][rosterIndex].charAt(0)=='*' ? false : true; 
        }
    }
    rosterEntriesReceived[rosterIndex] = true; 

    rosterCounter++;
    if (delegate) {
        bool fullyReceived = true;
        for (int i=0; i<rosterNumberOfEntries; i++) {
            if (!rosterEntriesReceived[i]) {
                fullyReceived = false;
                break;
            }
        }

        if (fullyReceived) {  // have them all now
            if (logLevel>0) console->println("WiT+DccEx:: dccExProcessRosterEntry(): all roster entries received");
            for (int i=0; i<rosterNumberOfEntries; i++) {
                if (delegate) delegate->receivedRosterEntry(i, rosterNames[i], rosterIds[i], (rosterIds[i]<127) ? 'S' : 'L');
            }
        }
    }
}

// DONE
void WitPlusDccEx::clearDccExRoster() {
    if (logLevel>0) console->println("WiT+DccEx:: clearDccExRoster()");
    rosterIds.clear();
    rosterAddresses.clear();
    rosterNames.clear();
    rosterEntriesReceived.clear();
    for (int k=0; k<MAX_FUNCTIONS; k++) {
        // if (logLevel>0) { console->print("WiT+DccEx:: clearDccExRoster() k: "); console->println(k); }
        rosterFunctionLabels[k].clear();
        rosterFunctionIsLatching[k].clear();
    }
}

// DONE
int WitPlusDccEx::getLocoIndexInRoster(String address) {
    int rosterId = -1;
    if (address[0] == 'S' || address[0] == 'L') {
        rosterId = address.substring(1).toInt();
    } else {
        rosterId = address.toInt();
    }

    return getLocoIndexInRoster(rosterId);
}

// DONE
int WitPlusDccEx::getLocoIndexInRoster(int rosterId) {
    if (logLevel>0) { console->print("WiT+DccEx:: getLocoIndexInRoster(): rosterNumberOfEntries:"); console->println(rosterNumberOfEntries); }
    for(int i=0;i<rosterNumberOfEntries;i++) {
        // if (logLevel>0) { console->print("WiT+DccEx:: getLocoIndexInRoster(): rosterIds[i]:"); console->println(rosterIds[i]); }
    
        if (rosterIds[i] == rosterId) {
            return i;
        }
    } 
    return -1;
}

// TODO - DELAY
void WitPlusDccEx::dccExProcessFastClock(char *c, int len) {
    if (logLevel>0) console->println("WiT+DccEx:: dccExProcessFastClock()");
}

// TODO
void WitPlusDccEx::dccExProcessRequestLocoId(char *c, int len) {
    if (logLevel>0) console->println("WiT+DccEx:: dccExProcessRequestLocoId()");
}

// DONE
// --- <= trackLetter state [cab]>
void WitPlusDccEx::dccExProcessTrackManager(char *c, int len) {
    if (logLevel>0) { console->print("WiT+DccEx:: dccExProcessTrackManager() c: "); console->println(c); }

    int start = 1; // space after command
    String trackString = getParameter(c, start, len, 1);
    // if (logLevel>0) { console->print("WiT+DccEx:: dccExProcessTrackManager() trackString: "); console->println(trackString); }

    int trackIndex = trackString.charAt(0) - 'A';

    // if (logLevel>0) { console->print("WiT+DccEx:: dccExProcessTrackManager() trackIndex: "); console->println(trackIndex); }

    if ( (trackIndex<0) || (trackIndex>7) ) return;

    String typeString = getParameter(c, start, len, 2);

    // if (logLevel>0) { console->print("WiT+DccEx:: dccExProcessTrackManager() typeString: "); console->println(typeString); }


    TrackType type = getTrackType(typeString);

    // ignore the DC address if there is one

    trackType[trackIndex] = type;

}

// ******************************************************************************************************

// NO CHANGE
bool WitPlusDccEx::addLocomotive(String address) {
    return addLocomotive(DEFAULT_MULTITHROTTLE, address);
}

// DONE - WiThrottle only
bool WitPlusDccEx::addLocomotive(char multiThrottle, String address) {
    if (isDccExServer()) return dccExSendAddLocomotive(multiThrottle, address);
       
    // WiThrottle protocol

    if (logLevel>0) { console->print("WiT+DccEx:: addLocomotive(): "); console->print(multiThrottle); console->print(" : "); console->println(address); }

    // int multiThrottleIndex = getMultiThrottleIndex(multiThrottle);
    bool ok = false;

    if (address[0] == 'S' || address[0] == 'L') {
        String rosterName = address; 
        String cmd;
        cmd = "M" + String(multiThrottle) + "+" + address + PROPERTY_SEPARATOR + rosterName;
        sendDelayedCommand(cmd);

        // boolean locoAlreadyInList = false;
        // for(int i=0;i<locomotives[multiThrottleIndex].size();i++) {
        //     if (locomotives[multiThrottleIndex][i].equals(address)) {
        //         locoAlreadyInList = true;
        //         break;
        //     }
        // } 
        // if (!locoAlreadyInList) {
        //     locomotives[multiThrottleIndex].push_back(address);
        //     currentAddress[multiThrottleIndex] = locomotives[multiThrottleIndex].front();
        //     locomotivesFacing[multiThrottleIndex].push_back(Forward);
        //     locomotiveSelected[multiThrottleIndex] = true;
        //     timeLastLocoAcquired = millis();
        // }
        addLocoToThrottle(multiThrottle, address);
        ok = true;
    }

    if (logLevel>1) { console->print("WiT+DccEx:: addLocomotive(): end : ");  console->println(ok); }
    return ok;
}

// ******************************************************************************************************

// NO CHANGE - WiThrottle only
bool WitPlusDccEx::stealLocomotive(String address) {
    return stealLocomotive(DEFAULT_MULTITHROTTLE, address);
}

// NO CHANGE - WiThrottle only
bool WitPlusDccEx::stealLocomotive(char multiThrottle, String address) {
    if (logLevel>0) { console->print("WiT+DccEx:: stealLocomotive(): "); console->print(multiThrottle); console->print(" : "); console->println(address); }

    bool ok = true;
    // MTSxxxx<;>xxxxx
    String cmd = "M" + String(multiThrottle) + "S" +address + PROPERTY_SEPARATOR + address;
    sendDelayedCommand(cmd);

    return ok;
}

// ******************************************************************************************************

// NO CHANGE
bool WitPlusDccEx::releaseLocomotive(String address) {
    return releaseLocomotive(DEFAULT_MULTITHROTTLE, address);
}

// DONE
bool WitPlusDccEx::releaseLocomotive(char multiThrottle, String address) {
    if (isDccExServer()) return dccExReleaseLocomotive(multiThrottle, address);
       
    // WiThrottle protocol
    if (logLevel>0) { console->print("WiT+DccEx:: releaseLocomotive(): "); console->print(multiThrottle); console->print(" : "); console->println(address); }

    // int multiThrottleIndex = getMultiThrottleIndex(multiThrottle);
    // MT-*<;>r
    String cmd = "M" + String(multiThrottle) + "-";
    cmd.concat(address);
    cmd.concat(PROPERTY_SEPARATOR);
    cmd.concat("r");
    sendDelayedCommand(cmd);

    // if (address.equals(ALL_LOCOS_ON_THROTTLE)) {
    //         locomotives[multiThrottleIndex].clear();
    //         locomotivesFacing[multiThrottleIndex].clear();
    // } else {
    //     for(int i=0;i<locomotives[multiThrottleIndex].size();i++) {
    //         if (locomotives[multiThrottleIndex][i].equals(address)) {
    //             locomotives[multiThrottleIndex].erase(locomotives[multiThrottleIndex].begin()+i);
    //             locomotivesFacing[multiThrottleIndex].erase(locomotivesFacing[multiThrottleIndex].begin()+i);
    //             break;
    //         }
    //     } 
    // }
    
    // if (locomotives[multiThrottleIndex].size()==0) { 
    //     locomotiveSelected[multiThrottleIndex] = false;
    //     currentAddress[multiThrottleIndex] = "";
    // } else {        
    //     currentAddress[multiThrottleIndex] = locomotives[multiThrottleIndex].front();
    // }
    removeLocoFromThrottle(multiThrottle, address);

    if (logLevel>1) console->println("WiT+DccEx:: releaseLocomotive(): end"); 
    return true;
}

// ******************************************************************************************************

// NO CHANGE
String WitPlusDccEx::getLeadLocomotive() {
    return getLeadLocomotive(DEFAULT_MULTITHROTTLE);
}

// NO CHANGE
String WitPlusDccEx::getLeadLocomotive(char multiThrottle) {
    if (logLevel>0) { console->print("WiT+DccEx:: getLeadLocomotive(): "); console->println(multiThrottle); }

    int multiThrottleIndex = getMultiThrottleIndex(multiThrottle);
    if (locomotives[multiThrottleIndex].size()>0) { 
        return locomotives[multiThrottleIndex].front();
    }
    return {};
}

// ******************************************************************************************************

// NO CHANGE
String WitPlusDccEx::getLocomotiveAtPosition(int position) {
    return getLocomotiveAtPosition(DEFAULT_MULTITHROTTLE, position);
}

// NO CHANGE
String WitPlusDccEx::getLocomotiveAtPosition(char multiThrottle, int position) {
    if (logLevel>3) { console->print("WiT+DccEx:: getLocomotiveAtPosition(): "); console->print(multiThrottle); console->print(" : "); console->println(position); }

    int multiThrottleIndex = getMultiThrottleIndex(multiThrottle);
    if (logLevel>3) { console->print("WiT+DccEx:: getLocomotiveAtPosition(): vector size: "); console->println(locomotives[multiThrottleIndex].size()); }
    if (locomotives[multiThrottleIndex].size()>0) { 
        if (logLevel>3) { console->print("WiT+DccEx:: getLocomotiveAtPosition(): return: "); console->println(locomotives[multiThrottleIndex][position]); }
        return locomotives[multiThrottleIndex][position];
    }
    return {};
}

// ******************************************************************************************************

// NO CHANGE
int WitPlusDccEx::getNumberOfLocomotives() {
    return getNumberOfLocomotives(DEFAULT_MULTITHROTTLE);
}

// NO CHANGE
int WitPlusDccEx::getNumberOfLocomotives(char multiThrottle) {
    if (logLevel>3) { console->print("WiT+DccEx:: getNumberOfLocomotives(): "); console->println(multiThrottle); }

    int multiThrottleIndex = getMultiThrottleIndex(multiThrottle);

    int size = locomotives[multiThrottleIndex].size();
    if (logLevel>3) { console->print("WiT+DccEx:: getNumberOfLocomotives(): end "); console->println(size); }
    return size;
}

// ******************************************************************************************************

// NO CHANGE
int WitPlusDccEx::getSpeedSteps() {
    return getSpeedSteps(DEFAULT_MULTITHROTTLE);
}

// NO CHANGE 
int WitPlusDccEx::getSpeedSteps(char multiThrottle) {
    if (logLevel>0) { console->print("WiT+DccEx:: getSpeedSteps(): "); console->println(multiThrottle); }

    int multiThrottleIndex = getMultiThrottleIndex(multiThrottle);
    return speedSteps[multiThrottleIndex];
}

// ******************************************************************************************************

// NO CHANGE 
bool WitPlusDccEx::setSpeedSteps(int steps) {
    return setSpeedSteps(DEFAULT_MULTITHROTTLE, steps);
}

// DONE
bool WitPlusDccEx::setSpeedSteps(char multiThrottle, int steps) {
    if (isDccExServer()) return setSpeedSteps(multiThrottle, steps);
       
    // WiThrottle protocol

    if (logLevel>0) { console->print("WiT+DccEx:: setSpeedSteps(): "); console->print(multiThrottle); console->print(" : "); console->println(steps); }

    int multiThrottleIndex = getMultiThrottleIndex(multiThrottle);

    // 1 = 128step, 2 = 28step, 4 = 27step or 8 = 14step
    if (steps != 1 && steps != 2 && steps != 4 && steps != 8) {
        console->print("WiT+DccEx:: setSpeedSteps(): Error, not one of the known values");
        return false;
    }

    String cmd = "M" + String(multiThrottle) + "A*" 
        + PROPERTY_SEPARATOR
        + "s"
        + String(steps);
    sendDelayedCommand(cmd);
    speedSteps[multiThrottleIndex] = steps;

    return true;
}


// ******************************************************************************************************

// NO CHANGE
bool WitPlusDccEx::setSpeed(int speed) {
    return setSpeed(DEFAULT_MULTITHROTTLE, speed);
}

// NO CHANGE
bool WitPlusDccEx::setSpeed(char multiThrottle, int speed) {
    return setSpeed(multiThrottle, speed, false);
}

// DONE  - TODO Check the need for Force Send for DCC-EX
bool WitPlusDccEx::setSpeed(char multiThrottle, int speed, bool forceSend) {
    if (isDccExServer()) return dccExSetSpeed(multiThrottle, speed);
       
    // WiThrottle protocol

    if (logLevel>0) { console->print("WiT+DccEx:: setSpeed(): "); console->print(multiThrottle); console->print(" : "); console->print(speed); console->print(" : "); console->println(forceSend); }

    int multiThrottleIndex = getMultiThrottleIndex(multiThrottle);
    if (speed < 0 || speed > 126) {
        return false;
    }
    if (!locomotiveSelected[multiThrottleIndex]) {
        return false;
    }

    if ( (speed != currentSpeed[multiThrottleIndex]) || (forceSend) ) {
        String cmd = "M" + String(multiThrottle) + "A*" 
            + PROPERTY_SEPARATOR
            + "V"
            + String(speed);
        sendDelayedCommand(cmd);
        currentSpeed[multiThrottleIndex] = speed;
    }
    return true;
}

// ******************************************************************************************************

// NO CHANGE
int WitPlusDccEx::getSpeed() {
    return getSpeed(DEFAULT_MULTITHROTTLE);
}

// NO CHANGE
int WitPlusDccEx::getSpeed(char multiThrottle) {
    if (logLevel>0) { console->print("WiT+DccEx:: getSpeed(): "); console->println(multiThrottle); }

    int multiThrottleIndex = getMultiThrottleIndex(multiThrottle);
    return currentSpeed[multiThrottleIndex];
}

// ******************************************************************************************************

// NO CHANGE
bool WitPlusDccEx::setDirection(Direction direction) {
    return setDirection(DEFAULT_MULTITHROTTLE, ALL_LOCOS_ON_THROTTLE, direction);
}

// NO CHANGE
bool WitPlusDccEx::setDirection(char multiThrottle, Direction direction) {
    return setDirection(multiThrottle, ALL_LOCOS_ON_THROTTLE, direction, false);
}

// NO CHANGE
bool WitPlusDccEx::setDirection(char multiThrottle, Direction direction, bool ForceSend) {
    return setDirection(multiThrottle, ALL_LOCOS_ON_THROTTLE, direction, ForceSend);
}

// NO CHANGE 
bool WitPlusDccEx::setDirection(char multiThrottle, String address, Direction direction) {
    return setDirection(multiThrottle, address, direction, false);
}

// DONE
bool WitPlusDccEx::setDirection(char multiThrottle, String address, Direction direction, bool forceSend) {
    if (isDccExServer()) return dccExSetDirection(multiThrottle, address, direction, forceSend);
       
    // WiThrottle protocol

    if (logLevel>0) { console->print("WiT+DccEx:: setDirection(): address: "); console->print(address); console->print(" throttle: "); 
    console->print(multiThrottle); console->print(" direction: "); console->println(direction); }

    int multiThrottleIndex = getMultiThrottleIndex(multiThrottle);
    if (!locomotiveSelected[multiThrottleIndex]) {
        return false;
    }

    String directionString = (direction == Reverse) ? "0" : "1";
    Direction currentDir = currentDirection[multiThrottleIndex];
    int locoIndex = -1;
    if (!address.equals(ALL_LOCOS_ON_THROTTLE)) {
        for(int i=0;i<locomotives[multiThrottleIndex].size();i++) {
            if (locomotives[multiThrottleIndex][i].equals(address)) {
                locoIndex = i;
                currentDir = locomotivesFacing[multiThrottleIndex][i];
                break;
            }
        }
    }

    if ( (direction != currentDir) || (forceSend) ) {
        String cmd = "M" + String(multiThrottle) + "A" + address + PROPERTY_SEPARATOR + "R" + directionString;
        sendDelayedCommand(cmd);

        if (locoIndex == -1) { // all locos
            currentDirection[multiThrottleIndex] = direction;
        } else {
            locomotivesFacing[multiThrottleIndex][locoIndex] = direction;
        }
    }
    return true;
}

// ******************************************************************************************************

// NO CHANGE
Direction WitPlusDccEx::getDirection() {
    return getDirection(DEFAULT_MULTITHROTTLE, ALL_LOCOS_ON_THROTTLE);
}

// NO CHANGE
Direction WitPlusDccEx::getDirection(char multiThrottle) {
    return getDirection(multiThrottle, ALL_LOCOS_ON_THROTTLE);
}

// NO CHANGE
Direction WitPlusDccEx::getDirection(char multiThrottle, String address) {
    if (logLevel>0) { console->print("WiT+DccEx:: getDirection(): addr: "); console->print(address); console->print(" throttle: "); console->println(multiThrottle); }

    int multiThrottleIndex = getMultiThrottleIndex(multiThrottle);

    if (address.equals(ALL_LOCOS_ON_THROTTLE)) {
        if (logLevel>0) { console->print("WiT+DccEx:: getDirection(): all dir: "); console->println(currentDirection[multiThrottleIndex]); }
        return currentDirection[multiThrottleIndex];
    } else {
        Direction individualDirection = currentDirection[multiThrottleIndex];
        for(int i=0;i<locomotives[multiThrottleIndex].size();i++) {
            if (locomotives[multiThrottleIndex][i].equals(address)) {
                individualDirection = locomotivesFacing[multiThrottleIndex][i];
                break;
            }
        }
        if (logLevel>0) { console->print("WiT+DccEx:: getDirection(): individual dir: "); console->println(individualDirection); }
        return individualDirection;
    }
}

// ******************************************************************************************************

// NO CHANGE
void WitPlusDccEx::emergencyStop() {
    emergencyStop('*', ALL_LOCOS_ON_THROTTLE);
}
// NO CHANGE
void WitPlusDccEx::emergencyStop(char multiThrottle) {
    emergencyStop(multiThrottle, ALL_LOCOS_ON_THROTTLE);
}

// DONE
void WitPlusDccEx::emergencyStop(char multiThrottle, String address) {
    if (logLevel>0) console->println("WiT+DccEx:: emergencyStop()");

    if (isDccExServer()) {
        dccExEmergencyStop(multiThrottle, address);
        return;
    }
       
    // WiThrottle protocol

    if (logLevel>0) { console->print("WiT+DccEx:: emergencyStop(): "); console->print(multiThrottle);console->print(" address: "); console->print(address);  }

    String cmd; cmd.reserve(10);
    char multiThrottleChar = multiThrottle;

    if (multiThrottleChar!='*') { // single throttle
        setSpeed(multiThrottle,0);
        cmd = "M" + String(multiThrottle) + "A" + address + PROPERTY_SEPARATOR + "X";
        sendDelayedCommand(cmd);
    } else { // all throttles
        for (int i=0; i<MAX_WIT_THROTTLES; i++) {
            multiThrottleChar = '0' + i;
            cmd = "M" + String(multiThrottleChar) + "A" + address + PROPERTY_SEPARATOR + "X";
            sendDelayedCommand(cmd);
        }
    }
}

// ******************************************************************************************************

// NO CHANGE
void WitPlusDccEx::setFunction(int funcNum, bool pressed) {
    setFunction(DEFAULT_MULTITHROTTLE, "", funcNum, pressed, false);
}

// NO CHANGE
void WitPlusDccEx::setFunction(char multiThrottle, int funcNum, bool pressed) {
    setFunction(multiThrottle, "", funcNum, pressed, false) ;
}

// NO CHANGE
void WitPlusDccEx::setFunction(char multiThrottle, String address, int funcNum, bool pressed) {
    setFunction(multiThrottle, address, funcNum, pressed, false) ;
}

// DONE - TODO is force needed in DCc-EX
void WitPlusDccEx::setFunction(char multiThrottle, String address, int funcNum, bool pressed, bool force) {
    if (logLevel>2) { console->print("WiT+DccEx:: setFunction() address: "); console->print(address); console->print(" funcNum: "); console->println(funcNum); }

    if (isDccExServer()) {
        dccExSetFunction(multiThrottle, address, funcNum, pressed);
        return;
    }
       
    // WiThrottle protocol

    if (logLevel>0) { console->print("WiT+DccEx:: setFunction(): "); console->print(multiThrottle); console->print(" : "); console->println(funcNum); }

    int multiThrottleIndex = getMultiThrottleIndex(multiThrottle);
    if (!locomotiveSelected[multiThrottleIndex]) {
        if (logLevel>0) console->println("WiT+DccEx:: setFunction(): end - not selected");
        return;
    }

    if (funcNum < 0 || funcNum > MAX_FUNCTIONS) {
        return;
    }

    String cmd = "M" + String(multiThrottle) + "A";
    if (address.equals("")) {
        cmd.concat(currentAddress[multiThrottleIndex]);
    } else {
        cmd.concat(address);
    }

    cmd.concat(PROPERTY_SEPARATOR);
    if (!force) {
        cmd.concat("F");
    } else {
        cmd.concat("f");
    }

    if (pressed) {
        cmd += "1";
    }
    else {
        cmd += "0";
    }
    cmd += funcNum;
    sendDelayedCommand(cmd);

    if (logLevel>1) console->println("WiT+DccEx:: setFunction(): end"); 
}

// ******************************************************************************************************

// DONE
void WitPlusDccEx::setTrackPower(TrackPower state) {
    if (isDccExServer()) {
        dccExSetTrackPower(state);
        return;
    }
       
    // WiThrottle protocol

    String cmd = "PPA";
    cmd.concat(state);

    sendDelayedCommand(cmd);	
}

// DONE
void WitPlusDccEx::setTrackPower(TrackPower state, char track) {
    if (isDccExServer()) dccExSetTrackPower(state, track);

    // else do nothing, as DCC-EX only support multiple tracks
}

// DONE
void WitPlusDccEx::setTrackPower(TrackPower state, String track) {
    if (isDccExServer()) dccExSetTrackPower(state, track);

    // else do nothing, as DCC-EX only support multiple tracks
}

// DONE
bool WitPlusDccEx::setTurnout(String turnoutSystemName, TurnoutAction action) {  // address is turnout system name
    if (isDccExServer()) return dccExSetTurnout(turnoutSystemName, action);

    // WiThrottle protocol
    String s = "T";
    if (action == TurnoutClose) {
        s = "C";
    } 
    else if (action == TurnoutToggle) {
        s = "2";
    }
    String cmd = "PTA" + s + turnoutSystemName;
    sendDelayedCommand(cmd);

    return true;
}

// DONE
bool WitPlusDccEx::setRoute(String routeSystemName) {  // address is turnout system name
    if (isDccExServer()) return dccExSetRoute(routeSystemName);

    // WiThrottle protocol
    String cmd = "PRA2" + routeSystemName;
    sendDelayedCommand(cmd);

    return true;
}

// NO CHANGE
long WitPlusDccEx::getLastServerResponseTime() {
  return lastServerResponseTime;   
}

