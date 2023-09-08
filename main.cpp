/**
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Created By Joseph Kelly
Created 13 July 2020

Instructions  : ***********************************************************
                 Create a program to generate emulated travel log outputs.
                 These travel logs must comply with the provided interface
                 description, and constraints described in:
                    ./Project Files/Assessment.pdf
                    ./Project Files/JNY ICD - v1.0.pdf
Expected Use  : ***********************************************************
                compile: "g++ main.cpp"
                execute: "Kelly_final.exe"
Expected Output : *********************************************************
                Journey Files:
                    ./<vehicle_id>.JNY
                    ./<vehicle_id>.JNY
                    ...
                Example: CAR.JNY
                    CAR,Campagnola,3860,5.18,6.4,13.2,Fiat,1981,SPORTS,REGULAR
                    40.1547,-105.174,0
                    40.2419,-105.174,555.195
                    40.3142,-105.174,881.861
                    40.3712,-105.174,1244.45
                    40.3816,-105.16,1312.42
                    40.4104,-105.154,1525.39
                    40.4172,-105.135,1612.5
                    40.4832,-105.065,1970.14
                    40.5245,-105.065,2150.06
                    40.563,-104.977,2605.91
Notes         : ***********************************************************
                Developer Coding Environment:
                   - Operating System: Windows_NT x64 10.0.18363
                   - VS Code Version: 1.47.0
                   - compiler: g++ (i686-posix-dwarf-rev0, Built by MinGW-W64 project) 8.1.0

                Potential for improvement:
                   - More efficient use of memory, references (pointers, etc.)
                   - Program logging and levels
                   - Doxygen formatted comments
                   - break monolithic file into functions and modules
                       - vehicles.h
                       - navigation.h
                   - initialize from config file
                       - vehicles.cfg
                       - geofences.cfg
                   - calibrate "current position" with "external" GPS connection
                   - path planning
                       - PID control
                           - mitigate bounce, windup, and overshoot
                       - obstacle avoidance
                       - intelligent bearing deviation approaching geofence
References    : ***********************************************************
                 Assignment Files
                    ./Project Files/Programming Assessment.pdf
                    ./Project Files/JNY ICD - v1.0.pdf
                 Real, Physical, Paper Books:
                    - C++ Pocket Reference
                 Various websites:
                    - C++ references
                    - Stackoverflow
                    - w3schools
                    - tutorialsPoint
History       : ***********************************************************
    13 July 2020: downloaded, set up C++ environment, refamiliarization (3 hrs)
                    - downloaded and unzipped project files, set up MinGW, refamiliarize with C++
    14 July 2020: added object and coordinate calculations (6 hrs) getting used to syntax
                    - originally thought about vehicle parent class and subclassing vehicles
                        - went with separate constructors based on vehicle definitions
    15 July 2020: added waypoint generation and history
                    - lots of randomization... just to generate some data ¯\_(ツ)_/¯
    16 July 2020: added geofence checks for boats
                    - started with simple "if less than", realized it is waaay more complicated
                        - ray-tracing, some linear algebra, pathing, etc.
    17 July 2020: TODO: adding reading configuration from files: vehicles, geofence polygons, etc.
                  TODO: adding program log files ()
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

#include <iostream>
#include <iomanip>
#include <sstream>
#include <algorithm>
#include <vector>
#include <list>
//#include <format> // "C++20"
#include <fstream>
#include <time.h>
#include <stdlib.h>
#include <math.h>

#include "GeoCalc.cpp"
//#include "point-in-poly.cpp"

//this could, and may already have, caused naming problems, a bit like "STX::LoadAll()"
using namespace std;

struct location {
    double latitude;
    double longitude;
};

struct historicPoint {
    location thisWaypoint;
    double elapsedTime;
};

struct geoFenceZone {
    location zoneNW;
    location zoneNE;
    location zoneSE;
    location zoneSW;
};


// === Boat Location Restrictions ===
location zone1NW = {56.2,-49.8};
location zone1NE = {56.2,-23.1};
location zone1SE = {15.6,-23.1};
location zone1SW = {15.6,-49.8};
geoFenceZone geoFenceZone1 = {zone1NW, zone1NE,
                                zone1SE, zone1SW};

location zone2NW = {-6.9,-28.6};
location zone2NE = {-6.9,8.2};
location zone2SE = {-48.8,8.2};
location zone2SW = {-48.8,-28.6};
geoFenceZone geoFenceZone2 = {zone2NW, zone2NE,
                                zone2SE, zone2SW};

location zone3NW = {8.1,-161.4};
location zone3NE = {8.1,-98.4};
location zone3SE = {-43.4,-98.4};
location zone3SW = {-43.4,-161.4};
geoFenceZone geoFenceZone3 = {zone3NW, zone3NE,
                                zone3SE, zone3SW};

location zone4NW = {-1.4,62.2};
location zone4NE = {-1.4,94.5};
location zone4SE = {-41.1,94.5};
location zone4SW = {-41.1,62.2};
geoFenceZone geoFenceZone4 = {zone4NW, zone4NE,
                                zone4SE, zone4SW};

std::list<geoFenceZone> boatZones = {geoFenceZone1,geoFenceZone2,
                                    geoFenceZone3,geoFenceZone4};

class Vehicle {
        // Record Position History
        double currentBearing;
        location currentLocation;
        std::list<struct historicPoint> pointsHistory;

        // Basic Vehicle properties
        string ident;
        string descrip;
        float weight;
        float width;
        float height;
        float length;

        // Additional Car:Vehicle properties
        string manufacturer;
        int year;
        string body_style;
        string fuel;
        //int unspecified0; // six fields indicated, four specified
        //int unspecified1; // six fields indicated, four specified

        // Additional Boat:Vehicle properties
        string powerType;
        float draftFt;

    public:
        //Destructor
        ~Vehicle()
        {
            /**
             * TODO: delete all the things here?
            */
        }

        //Boat constructor
        Vehicle(string ident, string descrip, float weight, float width, float height,
            float length, string powerType, float draftFt, string manufacturer){
            SetIdent(ident);
            SetDescrip(descrip);
            SetWeight(weight);
            SetWidth(width);
            SetHeight(height);
            SetLength(length);
            SetPowerType(powerType);
            SetDraft(draftFt);
            SetManufacturer(manufacturer);
            SetBearing(0.0);
            LogMessage(Identify(),true); //start/restart log
            srand (time(NULL)); // init rand at object creation
        }

        //Car constructor
        Vehicle(string ident, string descrip, float weight, float width, float height,
        float length, string manufacturer, int year, string body_style, string fuel){
            SetIdent(ident);
            SetDescrip(descrip);
            SetWeight(weight);
            SetWidth(width);
            SetHeight(height);
            SetLength(length);
            SetManufacturer(manufacturer);
            SetYear(year);
            SetBodyStyle(body_style);
            SetFuelType(fuel);
            SetBearing(0.0);
            LogMessage(Identify(),true); //start/restart log
            srand (time(NULL)); // init rand at object creation
            // SetUnspecified0(unspecified0);
            // SetUnspecified1(unspecified1);
        }
        // void SetUnspecified0(unsp){
        //     unspecified0 = unsp;
        // }
        // void SetUnspecified1(unsp){
        //     unspecified1 = unsp;
        // }
        // void GetUnspecified0(){
        //     return unspecified0;
        // }
        // void GetUnspecified1(){
        //     return unspecified1;
        // }

        //Generic, plain/"PLANE" constructor
        Vehicle(string ident, string descrip, float weight,
                float width, float height, float length){
            SetIdent(ident);
            SetDescrip(descrip);
            SetWeight(weight);
            SetWidth(width);
            SetHeight(height);
            SetLength(length);
            SetBearing(0.0);
            LogMessage(Identify(),true); //start/restart log
            srand (time(NULL)); // init rand at object creation
        }

        void SetIdent(string id){
            //ASCII char limit?  else throw ?
            ident = id;
        }

        void SetDescrip(string desc){
            //ASCII char limit?  else throw ?
            descrip = desc;
        }

        void SetWeight(float w){
            //non-negative, less than N=100,000,000 pounds?
            if (w >= 0){
                weight = w;
            }  // else throw ?
        }

        void SetWidth(float w){
            //non-negative, less than N=500 feet?
            if (w >= 0){
                width = w;
            } // else throw ?
        }

        void SetHeight(float h){
            //non-negative, less than N=500 feet?
            if (h >= 0){
                height = h;
            } // else throw?
        }

        void SetLength(float l){
            //non-negative, less than N=2000 feet?
            if (l >= 0){
                length = l;
            } // else throw?
        }

        void SetManufacturer(string manu){
            //ASCII char limit?  else throw ?
            manufacturer = manu;
        }

        void SetYear(float yr){
            /**
             * TODO: "this year", can't bet made "in the future"
             *        even if this program is still running in 50 years
            */
            if(yr > 1700 && yr < 2021){
                year = yr;
            } // else throw
        }

        void SetBodyStyle(string style){
            //Test for invalid body_style strings
            vector<string> valid_styles;
            valid_styles.push_back("COMPACT");
            valid_styles.push_back("COUPE");
            valid_styles.push_back("SEDAN");
            valid_styles.push_back("SPORTS");
            valid_styles.push_back("CROSSOVER");
            valid_styles.push_back("SUV");
            valid_styles.push_back("MINIVAN");
            valid_styles.push_back("VAN");
            valid_styles.push_back("TRUCK");
            valid_styles.push_back("BUS");
            valid_styles.push_back("SEMI");
            if (std::find(valid_styles.begin(), valid_styles.end(), style) != valid_styles.end()){
                body_style = style;
            } // else throw
        }

        void SetFuelType(string fl){
            //Test for invalid fuel_type strings
            vector<string> valid_fuels;
            valid_fuels.push_back("REGULAR");
            valid_fuels.push_back("DIESEL");
            valid_fuels.push_back("HYBRID");
            valid_fuels.push_back("ELECTRIC");
            if (std::find(valid_fuels.begin(), valid_fuels.end(), fl) != valid_fuels.end()){
                fuel = fl;
            } // else throw
        }

        void SetPowerType(string pwr){
            //Test for invalid power_options strings
            vector<string> valid_pwrOpts;
            valid_pwrOpts.push_back("UNPOWERED");
            valid_pwrOpts.push_back("SAIL");
            valid_pwrOpts.push_back("MOTOR");
            if (std::find(valid_pwrOpts.begin(), valid_pwrOpts.end(), pwr) != valid_pwrOpts.end()){
                powerType = pwr;
            } // else throw?
        }

        void SetDraft(float dft){
            //non-negative?, less than N feet?
            draftFt = dft;
        }

        void SetLocation(location point){
            //check is valid point for vehicle type?
            currentLocation = point;
        }

        void SetBearing(double aBearing){
            if(aBearing <= 360 && 0 <= aBearing){
                currentBearing = aBearing;
            }
        }

        void AddToWaypointHistory(location point, double epoch){
            historicPoint curPoint = { point, epoch };
            pointsHistory.push_back(curPoint); //most recent point at the end
            std::ostringstream stringStream;
            stringStream << curPoint.thisWaypoint.latitude << "," << curPoint.thisWaypoint.longitude << "," << curPoint.elapsedTime  << "\n";
            std::string msgCopyOfStr = stringStream.str();
            LogMessage(msgCopyOfStr);
        }

        void PrintWaypointHistory(){
            for (auto const& i : pointsHistory) {
                cout << i.thisWaypoint.latitude << "," << i.thisWaypoint.longitude << "," << i.elapsedTime  << "\n";
            }
        }

        double GetPreviousWaypointTime(){
            return pointsHistory.back().elapsedTime;
        }

        struct location GetLocation(){
            return currentLocation;
        }

        double GetBearing(){
            return currentBearing;
        }

        string GetManufacturer(){
            return manufacturer;
        }

        float GetYear(){
            return year;
        }

        string GetBodyStyle(){
            return body_style;
        }

        string GetFuelType(){
            return fuel;
        }

        string GetPowerType(){
            return powerType;
        }

        float GetDraft(){
            return draftFt;
        }

        string GetIdent(){
            return ident;
        }

        string GetDescrip(){
            return descrip;
        }

        float GetWeight(){
            return weight;
        }

        float GetWidth(){
            return width;
        }

        float GetHeight(){
            return height;
        }

        float GetLength(){
            return length;
        }

        string Identify(){
            ostringstream stringStream;
            if ("CAR" == GetIdent()){
                stringStream << GetIdent() << "," << GetDescrip() << "," << GetWeight() << "," << GetWidth() << "," << GetHeight() << "," << GetLength() << "," << GetManufacturer() << "," << GetYear() << "," << GetBodyStyle() << "," << GetFuelType() << "\n";
            } else if ("BOAT" == GetIdent()){
                stringStream << GetIdent() << "," << GetDescrip() << "," << GetWeight() << "," << GetWidth() << "," << GetHeight() << "," << GetLength() << "," << GetPowerType()  << "," << GetDraft()  << "," << GetManufacturer() << "\n";
            } else {
                stringStream << GetIdent() << "," << GetDescrip() << "," << GetWeight() << "," << GetWidth() << "," << GetHeight() << "," << GetLength() << "\n";
            }
            string msgCopyOfStr = stringStream.str();
            return msgCopyOfStr;
        }

        void LogMessage(string message, bool restartLog = false){
            if(restartLog){
                ofstream LogFile(GetIdent()+".JNY");
                LogFile << message;
                LogFile.close();
            } else {
                ofstream LogFile(GetIdent()+".JNY", std::ios_base::app); //append
                LogFile << message;
                LogFile.close();
            }
        }
};

double bearingGen(Vehicle someVehicle){
    /**
     * Some numerical linear algebra would be useful in calculating vectors
     * to add some intelligence - particularly with geoFences
     */

    string vehicle_type = someVehicle.GetIdent();
    double currentBearing = someVehicle.GetBearing();
    double newBearing;
    if(vehicle_type == "CAR"){
        // Car Bearing Sanity Check: Cars cannot turn more than 90 degrees between waypoints
        newBearing = currentBearing + ( rand() % 180 ) - 90;
        // if(abs(currentBearing-newBearing) > 90){
        //     cout << "\nCar Bearing:" << currentBearing << "," << newBearing << "\n";
        // } else throw?
    } else if (vehicle_type == "BOAT") {
        // Boat Bearing Sanity Check: Boats cannot turn more than 30 degrees between waypoints
        newBearing = currentBearing + ( rand() % 60 ) - 30;
        // if(abs(currentBearing-newBearing) > 30){
        //     cout << "\nBoat Bearing:" << currentBearing << "," << newBearing << "\n";
        // } else throw?
    }
    someVehicle.SetBearing(newBearing);
    return someVehicle.GetBearing();
}

double groundSpeedGen(Vehicle someVehicle){
     /**
      * Instantaneous velocity changes, controller windup,...
      * stoichiometry -> M mph -> N fps
      * If the vehicle is parked or unspecified
      * Vehicle does not HAVE to change speed every time
      * If the groundspeed goes negative, should the bearing change and then
      *     make the groundspeed positive again?
      */

    string vehicle_type = someVehicle.GetIdent();
    double mphToFps = 1.46666;
    double groundspeedFps = 0.0;
    if(vehicle_type == "CAR"){
        groundspeedFps = (rand() % 35 + 25)*mphToFps;
    } else if (vehicle_type == "BOAT") {
        string powerType = someVehicle.GetPowerType();
        if(powerType == "MOTOR"){
            groundspeedFps = (rand() % 35 + 25)*mphToFps;
        } else if (powerType == "SAIL")
        {
            groundspeedFps = (rand() % 15 + 15)*mphToFps;
        } else if (powerType == "UNPOWERED") {
            groundspeedFps = (rand() % 10 + 1)*mphToFps;
        }
    }
    // Not specified
    //else if (vehicle_type == "PLANE") {
    //     groundspeedFps = (rand() % 275 + 300)*mphToFps;
    // }
    return groundspeedFps;
}

bool geoFenceCheck(Vehicle someVehicle, location point){
    /**
     * Point-in-polygon (PIP) problem
     *  Test cases: inside, outside, edge
     * More numerical linear algebra
     *
     * References:
     *  https://www.tutorialspoint.com/Check-if-a-given-point-lies-inside-a-Polygon
     *      ^^^^ considering using this as a helper file instead of this geoFenceCheck function
     *  http://alienryderflex.com/polygon/
     *  https://www.codeproject.com/Articles/62482/A-Simple-Geo-Fencing-Using-Polygon-Method
     */

    if(abs(point.latitude) > 90 || abs(point.longitude) > 180){
        return false; // invalid location
    }
    if(someVehicle.GetIdent() == "BOAT"){
        //rudimentary check
        for (auto const& this_zone : boatZones) {
            bool lat_valid = false;
            bool lon_valid = false;
            //Check Latitude (north and south points)
            if (point.latitude >= 0){
                if (point.latitude < this_zone.zoneNW.latitude &&
                    this_zone.zoneSW.latitude < point.latitude){
                    lat_valid = true;
                }
            } else {
                if (point.latitude > this_zone.zoneNW.latitude &&
                    this_zone.zoneSW.latitude < point.latitude){
                    lon_valid = true;
                }
            }

            //Check Longitude (east and west points)
            if (point.longitude >= 0){
                if (this_zone.zoneNW.longitude > point.longitude &&
                    point.longitude > this_zone.zoneNE.longitude){
                    lat_valid = true;
                }
            } else {
                if (this_zone.zoneNW.longitude < point.longitude &&
                    point.longitude < this_zone.zoneNE.longitude){
                    lon_valid = true;
                }
            }
            if(lat_valid && lon_valid){
                return true;
            }
        }
        return false;
    } else {
        return true;
    }
}

void GenerateWaypointHistory(Vehicle aVehicle){
    srand (time(NULL));
    int num_waypoints = rand() % 20 + 10; // 10 to 30 waypoints, inclusive.

    for (int i = 0; i <= num_waypoints; i++){
        double endLatitude;
        double endLongitude;
        double startBearing;
        double distanceFeet;
        location vehicle_location = aVehicle.GetLocation();
        double startLatitude = vehicle_location.latitude;
        double startLongitude = vehicle_location.longitude;
        bool validLocation = false;

        /*
        Would be helped by some linear algebra ... rather than brute force
            randomly retrying bearings and directions to get a valid end location.
        */
        while (!validLocation){
            startBearing = bearingGen(aVehicle);
            distanceFeet = rand() % 221760;  // just going some random theoretical distance 0 to 49 miles

            //Provided in Project Files
            GeoCalc::GetEndingCoordinates(startLatitude, startLongitude,
                                            startBearing, distanceFeet,
                                            &endLatitude, &endLongitude);
            location vehicle_destination = {endLatitude, endLongitude};
            validLocation = geoFenceCheck(aVehicle, vehicle_destination);
        }

        //Provided in Project Files
        GeoCalc::GetGreatCircleDistance(startLatitude, startLongitude,
                                    endLatitude, endLongitude,
                                    &distanceFeet);

        double groundSpeedFps = groundSpeedGen(aVehicle);
        double segmentTravelTime = distanceFeet/groundSpeedFps;

        location thisPoint = {endLatitude, endLongitude};
        double elapsedTime = segmentTravelTime + aVehicle.GetPreviousWaypointTime();
        aVehicle.SetLocation(thisPoint);

        //Test: check elapsed time is always increasing
        aVehicle.AddToWaypointHistory(thisPoint,elapsedTime);
    }
}

int main(){
    Vehicle plane("PLANE","747",735000,195.66,63.413,231.82);
    Vehicle isidore("CAR","SVC00919",3860,5.18,6.4,13.2,"Fiat",1981,"SPORTS","REGULAR");
    Vehicle peters_barque("BOAT","Oceanis 60",48600,16.75,120.5,59.83,"SAIL",8.2,"Beneteau");

    location thisPoint = {40.154742, -105.173916};
    isidore.SetLocation(thisPoint);
    isidore.AddToWaypointHistory(thisPoint, 0.0);

    location thatPoint = {45.048124, -31.565813};
    peters_barque.SetLocation(thatPoint);
    peters_barque.AddToWaypointHistory(thatPoint,0.0);

    GenerateWaypointHistory(isidore);
    GenerateWaypointHistory(peters_barque);
}