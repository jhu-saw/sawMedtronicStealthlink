/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id$

  Author(s): Peter Kazanzides, Anton Deguet
  Created on: 2006

  (C) Copyright 2006-2011 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _mtsMedtronicStealthlink_h
#define _mtsMedtronicStealthlink_h

#include <cisstMultiTask/mtsTaskPeriodic.h>
#include <cisstMultiTask/mtsTransformationTypes.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>

// data types used for wrapper
#include <sawMedtronicStealthlink/mtsMedtronicStealthlinkTypes.h>

// Always include last
#include <sawMedtronicStealthlink/sawMedtronicStealthlinkExport.h>

// forward declarations of Stealthlink types
class AsCL_Client;
class mtsMedtronicStealthlink_AsCL_Utils;

class CISST_EXPORT mtsMedtronicStealthlink: public mtsTaskPeriodic
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

 protected:

    AsCL_Client * Client;
    mtsMedtronicStealthlink_AsCL_Utils * Utils;

    // State data
    mtsStealthTool ToolData;
    mtsStealthFrame FrameData;
    mtsStealthRegistration RegistrationData;
    mtsStealthProbeCal ProbeCal;

    // Other persistent data
    mtsDoubleVec SurgicalPlan;  // entry point + target point
    bool StealthlinkPresent;

    // Class used to store tool information using cisstParameterTypes
    class Tool
    {
        std::string StealthName;
        std::string InterfaceName;
    public:
        Tool(const std::string &stealthName, const std::string &interfaceName) :
            StealthName(stealthName), InterfaceName(interfaceName) {}
        ~Tool(void) {}
        const std::string & GetStealthName(void) const { return StealthName; }
        const std::string & GetInterfaceName(void) const { return InterfaceName; }
        prmPositionCartesianGet TooltipPosition;
        prmPositionCartesianGet MarkerPosition;
    };

    typedef std::vector<Tool *> ToolsContainer;
    ToolsContainer Tools;
    Tool * CurrentTool, * CurrentFrame;

    // Class used to store registration data
    class Registration
    {
    public:
        vctFrm3 Transformation;
        mtsBool Valid;
        mtsDouble PredictedAccuracy;
    };
    Registration RegistrationMember;

    // Class used to store exam info
    class ExamInformation
    {
    public:
        vctDouble3 VoxelScale;
        vctInt3 Size;
        bool Valid;
    };
    ExamInformation ExamInformationMember;
    void RequestExamInformation(void);

    // surgical plan
    void RequestSurgicalPlan(void);
    void GetSurgicalPlan(mtsDoubleVec & plan) const;

    /*! Mark all tool data as invalid */
    void ResetAllTools(void);

    /*! Find a tool using the stealh name */
    Tool * FindTool(const std::string & stealthName) const;

    /*! Add a tool using its stealth name and the name of the corresponding provided interface */
    Tool * AddTool(const std::string & stealthName, const std::string & interfaceName);

    void Init(void);

 public:
    mtsMedtronicStealthlink(const std::string & taskName, const double & periodInSeconds);
    mtsMedtronicStealthlink(const mtsTaskPeriodicConstructorArg & arg);
    ~mtsMedtronicStealthlink();

    void Startup(void) {}

    /*! Configure the Stealthlink interface using an XML file. If the
      XML file is not found, the system uses a default IP address
      (192.168.0.1) and does not predefine any provided interfaces for
      the tools. Note that if a tool is not pre-defined via the XML
      file, it can still be discovered at runtime and a provided
      interface will be dynamically added.
    */
    void Configure(const std::string & filename = "");

    void Run(void);

    void Cleanup(void);
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsMedtronicStealthlink);

#endif // _mtsMedtronicStealthlink_h
