import os
import os.path
import time
from collections import defaultdict

from HGA import distance_functions

from HGA.parse import parseCSV

startTime = time.time()

METERS_PER_MILE = 1609.34

# PROBLEM_TYPE
# 1 --> mFSTSP optimal
# 2 --> mFSTSP heuristic (will need other parameters)
problemTypeString = {1: "mFSTSP IP", 2: "mFSTSP Heuristic"}

NODE_TYPE_DEPOT = 0
NODE_TYPE_CUST = 1

TYPE_TRUCK = 1
TYPE_UAV = 2

MODE_CAR = 1
MODE_BIKE = 2
MODE_WALK = 3
MODE_FLY = 4

ACT_TRAVEL = 0
ACT_SERVE_CUSTOMER = 1
ACT_DONE = 2


# =============================================================


def make_dict():
    return defaultdict(make_dict)


# Usage:
# tau = defaultdict(make_dict)
# v = 17
# i = 3
# j = 12
# tau[v][i][j] = 44


class make_node:
    def __init__(
        self,
        nodeType,
        latDeg,
        lonDeg,
        altMeters,
        parcelWtLbs,
        serviceTimeTruck,
        serviceTimeUAV,
        address,
    ):
        # Set node[nodeID]
        self.nodeType = nodeType
        self.latDeg = latDeg
        self.lonDeg = lonDeg
        self.altMeters = altMeters
        self.parcelWtLbs = parcelWtLbs
        self.serviceTimeTruck = serviceTimeTruck  # [seconds]
        self.serviceTimeUAV = serviceTimeUAV  # [seconds]
        self.address = address  # Might be None

    def __repr__(self):
        return self.__dict__.__repr__()


class make_vehicle:
    def __init__(
        self,
        vehicleType,
        takeoffSpeed,
        cruiseSpeed,
        landingSpeed,
        yawRateDeg,
        cruiseAlt,
        capacityLbs,
        launchTime,
        recoveryTime,
        serviceTime,
        batteryPower,
        flightRange,
    ):
        # Set vehicle[vehicleID]
        self.vehicleType = vehicleType
        self.takeoffSpeed = takeoffSpeed
        self.cruiseSpeed = cruiseSpeed
        self.landingSpeed = landingSpeed
        self.yawRateDeg = yawRateDeg
        self.cruiseAlt = cruiseAlt
        self.capacityLbs = capacityLbs
        self.launchTime = launchTime  # [seconds].
        self.recoveryTime = recoveryTime  # [seconds].
        self.serviceTime = serviceTime
        self.batteryPower = batteryPower  # [joules].
        self.flightRange = flightRange  # 'high' or 'low'

    def __repr__(self):
        return self.__dict__.__repr__()


class make_travel:
    def __init__(
        self,
        takeoffTime,
        flyTime,
        landTime,
        totalTime,
        takeoffDistance,
        flyDistance,
        landDistance,
        totalDistance,
    ):
        # Set travel[vehicleID][fromID][toID]
        self.takeoffTime = takeoffTime
        self.flyTime = flyTime
        self.landTime = landTime
        self.totalTime = totalTime
        self.takeoffDistance = takeoffDistance
        self.flyDistance = flyDistance
        self.landDistance = landDistance
        self.totalDistance = totalDistance

    def __repr__(self):
        return self.__dict__.__repr__()


class MTSP:
    def __init__(self, locations_file, distance_matrix_file, vehicles_file, numUAVs):

        self.locations_file = locations_file
        self.vehicles_file = vehicles_file

        self.dist_matrix_file = distance_matrix_file

        # Define data structures
        self.node = {}
        self.vehicle = {}
        self.travel = defaultdict(make_dict)

        # Read data for node locations, vehicle properties, and travel time matrix of truck:
        self.readData(numUAVs)

        # Calculate travel times of UAVs (travel times of truck has already been read when we called the readData function)
        # NOTE:  For each vehicle we're going to get a matrix of travel times from i to j,
        # 		 where i is in [0, # of customers] and j is in [0, # of customers].
        # 		 However, tau and tauPrime let node c+1 represent a copy of the depot.
        for vehicleID in self.vehicle:
            if self.vehicle[vehicleID].vehicleType == TYPE_UAV:
                # We have a UAV (Note:  In some problems we only have a truck)
                for i in self.node:
                    for j in self.node:
                        if j == i:
                            self.travel[vehicleID][i][j] = make_travel(
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
                            )
                        else:
                            [
                                takeoffTime,
                                flyTime,
                                landTime,
                                totalTime,
                                takeoffDistance,
                                flyDistance,
                                landDistance,
                                totalDistance,
                            ] = distance_functions.calcMultirotorTravelTime(
                                self.vehicle[vehicleID].takeoffSpeed,
                                self.vehicle[vehicleID].cruiseSpeed,
                                self.vehicle[vehicleID].landingSpeed,
                                self.vehicle[vehicleID].yawRateDeg,
                                self.node[i].altMeters,
                                self.vehicle[vehicleID].cruiseAlt,
                                self.node[j].altMeters,
                                self.node[i].latDeg,
                                self.node[i].lonDeg,
                                self.node[j].latDeg,
                                self.node[j].lonDeg,
                                -361,
                                -361,
                            )
                            self.travel[vehicleID][i][j] = make_travel(
                                takeoffTime,
                                flyTime,
                                landTime,
                                totalTime,
                                takeoffDistance,
                                flyDistance,
                                landDistance,
                                totalDistance,
                            )

    def readData(self, numUAVs):
        # b)  tbl_vehicles.csv
        tmpUAVs = 0
        rawData = parseCSV(
            self.vehicles_file,
            returnJagged=False,
            fillerValue=-1,
            delimiter=",",
            commentChar="%",
        )
        for i in range(0, len(rawData)):
            vehicleID = int(rawData[i][0])
            vehicleType = int(rawData[i][1])
            takeoffSpeed = float(rawData[i][2])
            cruiseSpeed = float(rawData[i][3])
            landingSpeed = float(rawData[i][4])
            yawRateDeg = float(rawData[i][5])
            cruiseAlt = float(rawData[i][6])
            capacityLbs = float(rawData[i][7])
            launchTime = float(rawData[i][8])  # [seconds].
            recoveryTime = float(rawData[i][9])  # [seconds].
            serviceTime = float(rawData[i][10])  # [seconds].
            batteryPower = float(rawData[i][11])  # [joules].
            flightRange = str(rawData[i][12])  # 'high' or 'low'

            if vehicleType == TYPE_UAV:
                tmpUAVs += 1
                if tmpUAVs <= numUAVs:
                    self.vehicle[vehicleID] = make_vehicle(
                        vehicleType,
                        takeoffSpeed,
                        cruiseSpeed,
                        landingSpeed,
                        yawRateDeg,
                        cruiseAlt,
                        capacityLbs,
                        launchTime,
                        recoveryTime,
                        serviceTime,
                        batteryPower,
                        flightRange,
                    )
            else:
                self.vehicle[vehicleID] = make_vehicle(
                    vehicleType,
                    takeoffSpeed,
                    cruiseSpeed,
                    landingSpeed,
                    yawRateDeg,
                    cruiseAlt,
                    capacityLbs,
                    launchTime,
                    recoveryTime,
                    serviceTime,
                    batteryPower,
                    flightRange,
                )

        if tmpUAVs < numUAVs:
            print(
                "WARNING: You requested %d UAVs, but we only have data on %d UAVs."
                % (numUAVs, tmpUAVs)
            )
            print("\t We'll solve the problem with %d UAVs.  Sorry." % (tmpUAVs))

        # a)  tbl_locations.csv
        rawData = parseCSV(
            self.locations_file,
            returnJagged=False,
            fillerValue=-1,
            delimiter=",",
            commentChar="%",
        )
        for i in range(0, len(rawData)):
            nodeID = int(rawData[i][0])
            nodeType = int(rawData[i][1])
            latDeg = float(rawData[i][2])  # IN DEGREES
            lonDeg = float(rawData[i][3])  # IN DEGREES
            altMeters = float(rawData[i][4])
            parcelWtLbs = float(rawData[i][5])
            if len(rawData[i]) == 10:
                addressStreet = str(rawData[i][6])
                addressCity = str(rawData[i][7])
                addressST = str(rawData[i][8])
                addressZip = str(rawData[i][9])
                address = "%s, %s, %s, %s" % (
                    addressStreet,
                    addressCity,
                    addressST,
                    addressZip,
                )
            else:
                address = ""  # or None?

            serviceTimeTruck = self.vehicle[1].serviceTime
            if numUAVs > 0:
                serviceTimeUAV = self.vehicle[2].serviceTime
            else:
                serviceTimeUAV = 0

            self.node[nodeID] = make_node(
                nodeType,
                latDeg,
                lonDeg,
                altMeters,
                parcelWtLbs,
                serviceTimeTruck,
                serviceTimeUAV,
                address,
            )

        # c) tbl_truck_travel_data.csv
        if os.path.isfile(self.dist_matrix_file):
            # Travel matrix file exists
            rawData = parseCSV(
                self.dist_matrix_file, returnJagged=False, fillerValue=-1, delimiter=","
            )
            for i in range(0, len(rawData)):
                tmpi = int(rawData[i][0])
                tmpj = int(rawData[i][1])
                tmpTime = float(rawData[i][2])
                tmpDist = float(rawData[i][3])

                for vehicleID in self.vehicle:
                    if self.vehicle[vehicleID].vehicleType == TYPE_TRUCK:
                        self.travel[vehicleID][tmpi][tmpj] = make_travel(
                            0.0, tmpTime, 0.0, tmpTime, 0.0, tmpDist, 0.0, tmpDist
                        )

        else:
            # Travel matrix file does not exist
            print(
                "ERROR: Truck travel data is not available. Please provide a data matrix in the following format in a CSV file, and try again:\n"
            )
            print(
                "from node ID | to node ID | travel time [seconds] | travel distance [meters]\n"
            )
            exit()
