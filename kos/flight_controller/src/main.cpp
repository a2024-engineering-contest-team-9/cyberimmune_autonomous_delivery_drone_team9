#include "../include/mission.h"
#include "../../shared/include/initialization_interface.h"
#include "../../shared/include/ipc_messages_initialization.h"
#include "../../shared/include/ipc_messages_autopilot_connector.h"
#include "../../shared/include/ipc_messages_credential_manager.h"
#include "../../shared/include/ipc_messages_navigation_system.h"
#include "../../shared/include/ipc_messages_periphery_controller.h"
#include "../../shared/include/ipc_messages_server_connector.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

#define RETRY_DELAY_SEC 1
#define RETRY_REQUEST_DELAY_SEC 5
#define FLY_ACCEPT_PERIOD_US 500000

// custom varibles
const int DISTANCE = 10;
const int MAX_DRONE_SPEED = 2;
const int MAX_ALT = 300;
const int MIN_ALT = 250;
const int DROP_ALT = 40;
const int WAYPOINT_RADIUS = 3;
const int EARTH_RADIUS = 6371000;

enum BordersName
{
    left_external,
    right_external,
    up_external,
    down_external,
    left_internal,
    right_internal,
    down_internal,
    center_wall
};

/* struct BordersCoords {
    float left_external = 0.0f;
    float right_external = 0.0f;
    float up_external = 0.0f;
    float down_external = 0.0f;
    float down_internal = 0.0f;
    float left_internal = 0.0f;
    float right_internal = 0.0f;
    float center_wall = 0.0f;
    float border = 0.0f;
} */

struct DroneCoords{
    float currentLat = 0.f;
    float currentLong = 0.f;
};


int32_t startLong, startLat, startAlt;
int16_t timer = 0;

bool is_stoped = false;
bool is_drop = false;
bool is_barrier = false;
bool is_speed = false;
bool is_killed = false;

MissionCommand* commandsPoint = NULL;
uint32_t commandsCount = 0;

uint32_t dropWaypoint;
uint32_t barrierWaypoint;
uint32_t speedWaypoint;
uint32_t startWaypoint;
uint32_t endWaypoint;

double distanceBetweenPoint = 0.0;

uint32_t pointerWaypoint = 0;
uint32_t defaultAlt;
uint32_t barrierAlt;
uint32_t lantPrev, longPrev, altPrev = 0;

double getDistanseBetweenPoint(int32_t longPrev, int32_t latPrev, int32_t longCurr, int32_t latCurr);
double degToRad(double deg);
double convertCoords(int32_t coord);


void missionHoldPoint();
bool checkLand();
int32_t orvdCheckFlyAccepted();
int32_t orvdPauseControle();
void cargoControle();

void printCoordsConsole();
void speedPerSecControle();
void reachedWaypoint();
void parseMissionPoints();
bool barrierCheck();
bool speedTrackCheck();
int heightCheck();
bool cargoCheck();
void checkBooling();
void kill();


int sendSignedMessage(char *method, char *response, char *errorMessage, uint8_t delay)
{
    char message[512] = {0};
    char signature[257] = {0};
    char request[1024] = {0};
    snprintf(message, 512, "%s?%s", method, BOARD_ID);
    fprintf(stderr, "[%s] Info: Отправлен запрос прям ваще запрос\n", ENTITY_NAME);
    while (!signMessage(message, signature))
    {
        fprintf(stderr, "[%s] Info: Отправлен запрос в орвд\n", ENTITY_NAME);
        fprintf(stderr, "[%s] Warning: Failed to sign %s message at Credential Manager. Trying again in %ds\n", ENTITY_NAME, errorMessage, delay);
        sleep(delay);
    }
    snprintf(request, 1024, "%s&sig=0x%s", message, signature);

    while (!sendRequest(request, response))
    {
        fprintf(stderr, "[%s] Info: Ждем ответ\n", ENTITY_NAME);
        fprintf(stderr, "[%s] Warning: Failed to send %s request through Server Connector. Trying again in %ds\n", ENTITY_NAME, errorMessage, delay);
        sleep(delay);
    }

    uint8_t authenticity = 0;
    while (!checkSignature(response, authenticity) || !authenticity)
    {
        fprintf(stderr, "[%s] Info: Проверяем ответ\n", ENTITY_NAME);
        fprintf(stderr, "[%s] Warning: Failed to check signature of %s response received through Server Connector. Trying again in %ds\n", ENTITY_NAME, errorMessage, delay);
        sleep(delay);
    }

    return 1;
};

int sendCheckMessage(char *method, char *response, char *errorMessage, uint8_t delay)
{
    char message[512] = {0};
    char signature[257] = {0};
    char request[1024] = {0};
    snprintf(message, 512, "%s?%s", method, BOARD_ID);
    fprintf(stderr, "[%s] Info: Отправлен запрос прям ваще запрос\n", ENTITY_NAME);
    if (!signMessage(message, signature))
    {
        fprintf(stderr, "[%s] Warning: Failed to sign %s message at Credential Manager. Trying again in %ds\n", ENTITY_NAME, errorMessage, delay);
        return false;
    }

    snprintf(request, 1024, "%s&sig=0x%s", message, signature);
    fprintf(stderr, "[%s] Info: Отправлен запрос прям ваще запрос 2\n", ENTITY_NAME);

    if (!sendRequest(request, response))
    {
        fprintf(stderr, "[%s] Warning: Failed to send %s request through Server Connector. Trying again in %ds\n", ENTITY_NAME, errorMessage, delay);
        return false;
    }
    fprintf(stderr, "[%s] Info: Отправлен запрос прям ваще запрос 3\n", ENTITY_NAME);
    uint8_t authenticity = 0;
    if (!checkSignature(response, authenticity) || !authenticity)
    {
        fprintf(stderr, "[%s] Warning: Failed to check signature of %s response received through Server Connector. Trying again in %ds\n", ENTITY_NAME, errorMessage, delay);
        return false;
    }

    return 1;
};

// вычислить допустимые погрешности, заменить 10
/* BordersName checkBorders(const BordersCoords* bordersCoords, float currentLat, float currentLong, float border) {
    if (bordersCoords->left_external + border < currentLong) 
        return BordersName::left_external;
    if (bordersCoords->right_external + border < currentLong) 
        return BordersName::right_external;
    if (bordersCoords->down_external + border < currentLong) 
        return BordersName::down_external;
    if (bordersCoords->up_external + border < currentLong) 
        return BordersName::up_external;
    if (bordersCoords->left_internal + border < currentLong || bordersCoords->left_internal - border > currentLong)
        return BordersName::left_internal;
    if (bordersCoords->right_internal + border < currentLong || bordersCoords->right_internal - border > currentLong)
        return BordersName::right_internal;
    if (bordersCoords->down_internal + border < currentLong || bordersCoords->down_internal - border > currentLong)
        return BordersName::down_internal;
    if (bordersCoords->center_wall + border < currentLong || bordersCoords->center_wall - border > currentLong)
        return BordersName::center_wall;
}

void updateCoords(DroneCoords* droneCoords, float lastLat, float lastLong)
{
    droneCoords->currentLong = lastLong * 1e-7f;
    droneCoords->currentLat = lastLat * 1e-7f;
}; */

void parseMissionPoints(){
    commandsPoint = getMissionCommands();
    commandsCount = getCommandsNum();
    printMission();
    defaultAlt = commandsPoint[1].content.waypoint.latitude;
    startWaypoint = 2;
    for (int i = 0; i < commandsCount; i++) {
        switch (commandsPoint[i].type) {
        case CommandType::WAYPOINT:
            if(commandsPoint[i].content.waypoint.altitude < defaultAlt && commandsPoint[i].content.waypoint.altitude > 30) {
                barrierWaypoint = i;
                barrierAlt = commandsPoint[i].content.waypoint.altitude;
                fprintf(stderr, "[%s] Info: Barrier: %d i: %d\n", ENTITY_NAME, commandsPoint[i].content.waypoint.altitude, i);
            }
            break;
        case CommandType::SET_SERVO:
            if(commandsPoint[i].content.servo.number==5){
                fprintf(stderr, "[%s] Info: Set servo: cargo i: %d\n", ENTITY_NAME, i);
                dropWaypoint = i;
            }
            break;
        }
    }
    endWaypoint = commandsCount-1;
    speedWaypoint = 7;
};

int main(void)
{
    // Before do anything, we need to ensure, that other modules are ready to work
    while (!waitForInit("periphery_controller_connection", "PeripheryController"))
    {
        fprintf(stderr, "[%s] Warning: Failed to receive initialization notification from Periphery Controller. Trying again in %ds\n", ENTITY_NAME, RETRY_DELAY_SEC);
        sleep(RETRY_DELAY_SEC);
    }
    while (!waitForInit("autopilot_connector_connection", "AutopilotConnector")) {
        fprintf(stderr, "[%s] Warning: Failed to receive initialization notification from Autopilot Connector. Trying again in %ds\n", ENTITY_NAME, RETRY_DELAY_SEC);
        sleep(RETRY_DELAY_SEC);
    }
    while (!waitForInit("navigation_system_connection", "NavigationSystem")) {
        fprintf(stderr, "[%s] Warning: Failed to receive initialization notification from Navigation System. Trying again in %ds\n", ENTITY_NAME, RETRY_DELAY_SEC);
        sleep(RETRY_DELAY_SEC);
    }
    while (!waitForInit("server_connector_connection", "ServerConnector")) {
        fprintf(stderr, "[%s] Warning: Failed to receive initialization notification from Server Connector. Trying again in %ds\n", ENTITY_NAME, RETRY_DELAY_SEC);
        sleep(RETRY_DELAY_SEC);
    }
    while (!waitForInit("credential_manager_connection", "CredentialManager")) {
        fprintf(stderr, "[%s] Warning: Failed to receive initialization notification from Credential Manager. Trying again in %ds\n", ENTITY_NAME, RETRY_DELAY_SEC);
        sleep(RETRY_DELAY_SEC);
    }
    fprintf(stderr, "[%s] Info: Initialization is finished\n", ENTITY_NAME);

    setKillSwitch(true);

    //Enable buzzer to indicate, that all modules has been initialized
    if (!enableBuzzer())
        fprintf(stderr, "[%s] Warning: Failed to enable buzzer at Periphery Controller\n", ENTITY_NAME);

    //Copter need to be registered at ORVD
    char authResponse[1024] = {0};
    sendSignedMessage("/api/auth", authResponse, "authentication", RETRY_DELAY_SEC);
    fprintf(stderr, "[%s] Info: Successfully authenticated on the server\n", ENTITY_NAME);

    //Constantly ask server, if mission for the drone is available. Parse it and ensure, that mission is correct
    while (true) {
        char missionResponse[1024] = {0};
        if (sendSignedMessage("/api/fmission_kos", missionResponse, "mission", RETRY_DELAY_SEC) && parseMission(missionResponse)) {
            fprintf(stderr, "[%s] Info: Successfully received mission from the server\n", ENTITY_NAME);
            printMission();
            break;
        }
        sleep(RETRY_REQUEST_DELAY_SEC);
    }

    //The drone is ready to arm
    fprintf(stderr, "[%s] Info: Ready to arm\n", ENTITY_NAME);
    while (true) {
        //Wait, until autopilot wants to arm (and fails so, as motors are disabled by default)
        while (!waitForArmRequest()) {
            fprintf(stderr, "[%s] Warning: Failed to receive an arm request from Autopilot Connector. Trying again in %ds\n", ENTITY_NAME, RETRY_DELAY_SEC);
            sleep(RETRY_DELAY_SEC);
        }
        fprintf(stderr, "[%s] Info: Received arm request. Notifying the server\n", ENTITY_NAME);

        //When autopilot asked for arm, we need to receive permission from ORVD
        char armRespone[1024] = {0};
        sendSignedMessage("/api/arm", armRespone, "arm", RETRY_DELAY_SEC);

        if (strstr(armRespone, "$Arm: 0#") != NULL) {
            //If arm was permitted, we enable motors
            fprintf(stderr, "[%s] Info: Arm is permitted\n", ENTITY_NAME);
            while (!setKillSwitch(true)) {
                fprintf(stderr, "[%s] Warning: Failed to permit motor usage at Periphery Controller. Trying again in %ds\n", ENTITY_NAME, RETRY_DELAY_SEC);
                sleep(RETRY_DELAY_SEC);
            }
            if (!permitArm())
                fprintf(stderr, "[%s] Warning: Failed to permit arm through Autopilot Connector\n", ENTITY_NAME);
            break;
        }
        else if (strstr(armRespone, "$Arm: 1#") != NULL) {
            fprintf(stderr, "[%s] Info: Arm is forbidden\n", ENTITY_NAME);
            if (!forbidArm())
                fprintf(stderr, "[%s] Warning: Failed to forbid arm through Autopilot Connector\n", ENTITY_NAME);
        }
        else
            fprintf(stderr, "[%s] Warning: Failed to parse server response\n", ENTITY_NAME);
        fprintf(stderr, "[%s] Warning: Arm was not allowed. Waiting for another arm request from autopilot\n", ENTITY_NAME);
    };

    // If we get here, the drone is able to arm and start the mission
    // The flight is need to be controlled from now on
    // Also we need to check on ORVD, whether the flight is still allowed or it is need to be paused
    getCoords(startLong, startLat, startAlt);
    parseMissionPoints();
    setCargoLock(0);
    is_drop = false;
    is_killed = false;
    
    /* DroneCoords droneCoords;
    BordersCoords bordersCoords;
    updateCoords(&droneCoords, static_cast<float>(startLat), static_cast<float>(startLong)); */
    while (true)
    {
        /* updateCoords(&droneCoords, droneCoords.currentLat, droneCoords.currentLong);
        if(!checkBorders(&bordersCoords, droneCoords.currentLat, droneCoords.currentLong))
            returnToBorders(); */

        fprintf(stderr, "[%s] is_koll : [%s]\n", ENTITY_NAME, is_killed ? "true" : "false");

        printCoordsConsole();

        /*heightCheck();*/
        reachedWaypoint();        

        orvdPauseControle();

        cargoControle();
        
        checkBooling();

        speedPerSecControle();

        
        /* if(!heightCheck()) {
            kill();
            fprintf(stderr, "[%s]KILLLLL\n", ENTITY_NAME);
        }; */

        

        // missionHoldPoint();

        /*if(!permitArm()){
        fprintf(stderr, "[%s] asdas;lfasdfl");
        }*/

        sleep(1);
    };
        

    return EXIT_SUCCESS;
}

double getDistanseBetweenPoint(int32_t longPrev, int32_t latPrev, int32_t longCurr, int32_t latCurr)
{
    double piPrev, piCurr, dPi, dL, a, b;
    piPrev = degToRad(convertCoords(latPrev));
    piCurr = degToRad(convertCoords(latCurr));
    dPi = degToRad(convertCoords(latCurr) - convertCoords(latPrev));
    dL = degToRad(convertCoords(longCurr) - convertCoords(longPrev));
    b = asin(sqrt(0.5*(1-cos(dPi)+cos(piPrev)*cos(piCurr)*(1-cos(dL)))));
    return 2 * EARTH_RADIUS* b;
};

void kill() {
    if(!is_killed){
        is_killed = true;
        setKillSwitch(true);
        fprintf(stderr, "Info: KILLED");
    }
}

double convertCoords(int32_t coord) {
    return coord * 1e-7;
}

double degToRad(double deg)
{
    return deg * M_PI / 180;
};

void printCoordsConsole()
{
    int32_t lantitude, longitude, altitude;
    getCoords(lantitude, longitude, altitude);
    fprintf(stderr, "Info: [%d].[%d] : [%d]\n", lantitude, longitude, altitude);
};

int32_t orvdPauseControle()
{
    if (!orvdCheckFlyAccepted())
    {
        if (!is_stoped)
        {
            is_stoped = true;
            pauseFlight();
        }
        return 1;
    }
    else
    {
        if (is_stoped)
        {
            is_stoped = false;
            resumeFlight();
        }
        return 0;
    }
};

bool barrierCheck() {
    if (pointerWaypoint == barrierWaypoint || pointerWaypoint == barrierWaypoint+1) {
        return true;
    }
    return false;
}

bool speedTrackCheck() {
    if (pointerWaypoint == speedWaypoint ) {
        return true;
    }
    return false;
};

void checkBooling() {
    fprintf(stderr, "[%s] kill: Попали в убивцу\n", ENTITY_NAME);
    if(!is_killed && pointerWaypoint >= 13) {
        int32_t curLong, curLat, curAlt;
        double distanceBetweenPointCurr;
        getCoords(curLat, curLong, curAlt);
        fprintf(stderr, "[%s] kill: Попали в убивцу внутри\n", ENTITY_NAME);
        distanceBetweenPointCurr = getDistanseBetweenPoint(commandsPoint[(uint32_t)13].content.waypoint.longitude, commandsPoint[(uint32_t)13].content.waypoint.latitude, curLong, curLat);
        
        if(distanceBetweenPoint == 0.0) {
            distanceBetweenPoint = distanceBetweenPointCurr;
        }


        fprintf(stderr, "[%s] kill lon lat: [%d] [%d]\n", ENTITY_NAME, commandsPoint[(uint32_t)13].content.waypoint.longitude, commandsPoint[(uint32_t)13].content.waypoint.latitude);
        fprintf(stderr, "[%s] kill curr lon lat: [%d] [%d]\n", ENTITY_NAME, curLong, curLat);
        fprintf(stderr, "[%s] kill: [%f] [%f]\n", ENTITY_NAME, distanceBetweenPoint, distanceBetweenPointCurr);


        if(distanceBetweenPointCurr - distanceBetweenPoint  >= 1.0){
            fprintf(stderr, "[%s] kill: [%f] [%f]\n", ENTITY_NAME, distanceBetweenPoint, distanceBetweenPointCurr);

            fprintf(stderr, "[%s] kill: \n", ENTITY_NAME);
            kill();
            return;
        }
        distanceBetweenPoint = distanceBetweenPointCurr;
    }
}

bool cargoCheck() {
    if(pointerWaypoint == dropWaypoint-1 || pointerWaypoint == dropWaypoint+1 || pointerWaypoint == dropWaypoint) {
        int32_t curLong, curLat, curAlt;
        double distanceCargoDrop;
        getCoords(curLat, curLong, curAlt);

        fprintf(stderr, "[%s] drop: На пути к поинту сброса\n", ENTITY_NAME);

        distanceCargoDrop = getDistanseBetweenPoint(curLong, curLat, commandsPoint[dropWaypoint-1].content.waypoint.longitude, commandsPoint[dropWaypoint-1].content.waypoint.latitude);
    
        if(pointerWaypoint == dropWaypoint+1){
            if(distanceCargoDrop <= 1.0) {
                return true;
            }
            fprintf(stderr, "[%s] drop: Должен офнуть [%s]\n", ENTITY_NAME, !is_drop ? "false" : "true");
            return false;
        }

        if(distanceCargoDrop <= 5.0) {
            fprintf(stderr, "[%s] drop: Подлетел к поинту сброса\n", ENTITY_NAME);
            return true;
        }
    }
    return false;
}


void reachedWaypoint() {
    int32_t lat, lon, alt;
    getCoords(lat, lon, alt);
    while((commandsPoint[pointerWaypoint].type != CommandType::WAYPOINT || (commandsPoint[pointerWaypoint].content.waypoint.longitude == 0 && commandsPoint[pointerWaypoint].content.waypoint.latitude == 0)) && pointerWaypoint != commandsCount){
        ++pointerWaypoint;
    }
    double distanceToWaypoint = getDistanseBetweenPoint(commandsPoint[pointerWaypoint].content.waypoint.longitude, commandsPoint[pointerWaypoint].content.waypoint.latitude,  lon, lat);
    fprintf(stderr, "[%s] distance: %f i:%d  \n", ENTITY_NAME, distanceToWaypoint, pointerWaypoint );
    fprintf(stderr, "[%d]:[%d] --- [%d]:[%d] \n", commandsPoint[pointerWaypoint].content.waypoint.longitude, commandsPoint[pointerWaypoint].content.waypoint.latitude,  lon, lat);
    if (distanceToWaypoint <= WAYPOINT_RADIUS) {
        fprintf(stderr, "[%s] Info: Waypoint %u reached\n", ENTITY_NAME, pointerWaypoint);
        ++pointerWaypoint;
    }
}

void missionHoldPoint()
{

    /*  if (!orvdCheckFlyAccepted())
    {
        if (!is_stoped)
        {
            is_stoped = true;
            pauseFlight();
        }
        if (is_stoped && timer <= 15)
        {
            timer = timer + 1;
        }

        if (timer == 15)
        {
            is_stoped = false;
            resumeFlight();
        }
    }
    else
    {
        if (is_stoped)
        {
            timer = 0;
            is_stoped = false;
            resumeFlight();
        }
    } */

    /* char armRespone[1024] = {0};
    sendCheckMessage("/api/arm", armRespone, "0", RETRY_DELAY_SEC);
    fprintf(stderr, "[%s] Info: Arm is opopo [%s]\n ", ENTITY_NAME, armRespone);
    if (timer <= 15)
    {
        if (strstr(armRespone, "$Arm: 0#") != NULL && is_stoped)
        {
            timer = 0;
            changeSpeed(2);
            is_stoped = false;
            fprintf(stderr, "[%s] Info: Arm is kakaka\n", ENTITY_NAME);
        }
        else if (strstr(armRespone, "$Arm: 1#") != NULL && !is_stoped)
        {
            is_stoped = true;
            changeSpeed(0);
            timer = timer + RETRY_DELAY_SEC;
            fprintf(stderr, "[%s] Info: Arm is stop\n", ENTITY_NAME);
        };
    }
    else
    {
        pauseFlight();
        fprintf(stderr, "[%s] Info: Arm is opopo [%s] : [%s]\n ", ENTITY_NAME,
                strstr(armRespone, "$Arm: 0#"), timer);
    } */
};

int32_t orvdCheckFlyAccepted()
{
    char armRespone[1024] = {0};
    sendCheckMessage("/api/fly_accept", armRespone, "arm", RETRY_DELAY_SEC);
    fprintf(stderr, "[%s] Info: Arm is opopo [%s]\n ", ENTITY_NAME, armRespone);
    if (strstr(armRespone, "$Arm: 0#") != NULL)
    {
        fprintf(stderr, "[%s] Info: Arm is available\n", ENTITY_NAME);
        return 1;
    }
    else if (strstr(armRespone, "$Arm: 1#") != NULL)
    {
        fprintf(stderr, "[%s] Info: Arm is stop\n", ENTITY_NAME);
        return 0;
    };
    return 1;
};


void speedPerSecControle()
{
    int32_t lantCurr, longCurr, altCurr;
    double speedCheck;
    getCoords(lantCurr, longCurr, altCurr);
    speedCheck = getDistanseBetweenPoint(longPrev, lantPrev, longCurr, lantCurr);
    lantPrev = lantCurr;
    longPrev = longCurr;
    altPrev = altCurr;

    if(is_drop) {
        changeSpeed(1);
        return;
    }
    if(speedTrackCheck()){
        if((uint32_t)round(speedCheck) > 5) {
            changeSpeed(5);
        }
        if((uint32_t)round(speedCheck) < 5) {
            changeSpeed(5);
        }
        return;
    }
    if (speedCheck > MAX_DRONE_SPEED)
    {
        changeSpeed(MAX_DRONE_SPEED);
    }
    if (speedCheck < MAX_DRONE_SPEED)
    {
        changeSpeed(MAX_DRONE_SPEED);
    }

    fprintf(stderr, "[%s] speed: %f  [%d]:[%d]\n", ENTITY_NAME, speedCheck, lantPrev, lantCurr);

    return;
} 

void cargoControle() {
    if(cargoCheck()){
        if(!is_drop){
            setCargoLock(1);
            is_drop = true;
        }
        return;
    };
    if(is_drop) {
        setCargoLock(0);
        is_drop = false;
        return;
    }
}

/* bool checkLand()
{
    int32_t lant, lon, alt;
    getCoords(lant, lon, alt);
    if (alt <= startAlt + 0.02)
    {
        return true;
    }
    return false;
} */

int heightCheck()
{

    int32_t lant, lon, alt;
    getCoords(lant, lon, alt);

    if(pointerWaypoint == commandsCount) {
        fprintf(stderr, "[%s] Последний поинт высота\n", ENTITY_NAME);
        if(!getCoords(lant, lon, alt)) {
            fprintf(stderr, "[%s] Заблокало\n", ENTITY_NAME);
        }
    }
    
    if(is_drop) {
        changeAltitude(50);
        return 1;
    }

    if(barrierCheck()){
        if (alt <= barrierAlt - 30 || alt >= barrierAlt + 30)
        {
           return changeAltitude(barrierAlt);
        }
        return 1;
    }


    if (alt > defaultAlt + 40)
    {
       return changeAltitude(defaultAlt);
    }
    if (alt < defaultAlt - 40)
    {
        return changeAltitude(defaultAlt);
    }
    return 1;
};
