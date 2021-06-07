//#include "src/CommandInterpreter.h"

#include <stdio.h>
#include <iostream>

#ifdef USE_SIM
#include <chrono>
#include <thread>
#include <nlohmann/json.hpp>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
namespace py = pybind11;


#include "DriveSystem.h"


nlohmann::json set_object_sphere_cmd = {
    {"kp", 14.0},
    {"kd", 2.0},
    {"max_current", 7.0},
};


void processMessageFromJsonString(const char* msg)
{
    nlohmann::json result = nlohmann::json::parse(msg);
    bool verbose = true;
    if (verbose) {
        std::cout << "received:" << result.dump(4) << std::endl;
    }


}

void processMessageFromMsgPack(const std::vector<unsigned char>& msg)
{
    nlohmann::json result = nlohmann::json::from_msgpack(msg, false);
    bool verbose = true;
    if (verbose) {
        std::cout << "received:" << result.dump(4) << std::endl;
    }
}

void setup()
{
    std::cout << "setup pupper_drive" << std::endl;
}

PYBIND11_MODULE(pupper_drive, m) {



  m.doc() = R"pbdoc(
        pupper drive system python plugin
        -----------------------

        .. currentmodule:: pupper_drive

        .. autosummary::
           :toctree: _generate

    )pbdoc";

  m.def("setup", &setup);
  m.def("processMessageFromMsgPack", &processMessageFromMsgPack);
  m.def("processMessageFromJsonString", &processMessageFromJsonString);


   py::class_<DriveSystem>(m, "DriveSystem")
      .def(py::init<>())
      //.def("set_zero", [](VectorX& a) {
      //    DriveSystem::
      //}

       .def("SetIdle", &DriveSystem::SetIdle)
       .def("Update", &DriveSystem::Update)
       .def("SetActivations", &DriveSystem::SetActivations)
       .def("CommandIdle", &DriveSystem::CommandIdle)
       .def("CommandCurrents", &DriveSystem::CommandCurrents)
       
       .def("GetLastCommandedCurrents", &DriveSystem::GetLastCommandedCurrents)

       .def("DefaultCartesianPositions", &DriveSystem::DefaultCartesianPositions)
       .def("SetDefaultCartesianPositions", &DriveSystem::SetDefaultCartesianPositions)
       
       .def("SetMeasuredPositions", &DriveSystem::SetMeasuredPositions)
       .def("SetMeasuredVelocities", &DriveSystem::SetMeasuredVelocities)
       

       .def("SetJointPositions", &DriveSystem::SetJointPositions)
       .def("SetPositionKp", &DriveSystem::SetPositionKp)
       .def("SetPositionKd", &DriveSystem::SetPositionKd)
       
       .def("SetCartesianKp", &DriveSystem::SetCartesianKp)
       .def("SetCartesianKd", &DriveSystem::SetCartesianKd)
       .def("SetCartesianKp3x3", &DriveSystem::SetCartesianKp3x3)
       .def("SetCartesianKd3x3", &DriveSystem::SetCartesianKd3x3)
       .def("SetCartesianPositions", &DriveSystem::SetCartesianPositions)
       .def("SetCartesianVelocities", &DriveSystem::SetCartesianVelocities)
       .def("SetFeedForwardForce", &DriveSystem::SetFeedForwardForce)
       .def("SetCurrent", &DriveSystem::SetCurrent)
       .def("SetFaultCurrent", &DriveSystem::SetFaultCurrent)
       .def("SetMaxCurrent", &DriveSystem::SetMaxCurrent)
       .def("CartesianPositionControl", &DriveSystem::CartesianPositionControl2)
       
       

       
       
       
       
       

       ;

  
  
#ifdef VERSION_INFO
  m.attr("__version__") = VERSION_INFO;
#else
  m.attr("__version__") = "dev";
#endif


  
#ifdef VERSION_INFO
  m.attr("__version__") = VERSION_INFO;
#else
  m.attr("__version__") = "dev";
#endif

}

int main(int argc, char* argv[])
{
    setup();
}

#endif