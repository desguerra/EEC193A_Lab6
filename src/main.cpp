#include "../include/plotter.h"
#include "../include/mapper.h"
#include "ekfslam.h"
#include <iostream>

MeasurementPackage initMesurement(const string& filename) {
    cout << "##### Init Measurement #####" << endl;
    MeasurementPackage measurement = MeasurementPackage();
    measurement.initialize(filename);
    return measurement;
}

Mapper initMapper(const string& filename) {
    cout << "##### Init Mapper #####" << endl;
    Mapper mapper = Mapper();
    mapper.initialize(filename);
    return mapper;
}

int main(int argc, char* argv[]) {
    cout << "##### EFK SLAM #####" << endl;
    if (argc != 3) {
        cerr << "Incorrect Number of command line argument. Expecting 3" << endl;
        cerr << "Run ./bin/EKFSLAM <path-to-sensor> <path-to-world> " << endl;
        exit(EXIT_FAILURE);
    }

    MeasurementPackage measurement = initMesurement(argv[1]);
    Mapper mapper = initMapper(argv[2]);

    for (MapPoint point : mapper.data) {
        cout << "Id: " << point.id
             << " x: "  << point.x
             << " y: "  << point.y << endl;
    }

    return 0;
}