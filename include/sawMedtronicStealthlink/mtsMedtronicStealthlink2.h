/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id: mtsMedtronicStealthlink.h 3754 2012-07-27 15:06:53Z dmirota1 $

  Author(s): Peter Kazanzides, Anton Deguet, Daniel Mirota
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

#include <cisstMultiTask/mtsTaskFromSignal.h>
#include <cisstMultiTask/mtsTransformationTypes.h>
#include <cisstMultiTask/mtsFixedSizeVectorTypes.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>

// data types used for wrapper
#include <sawMedtronicStealthlink/mtsMedtronicStealthlink2Types.h>

// Always include last
#include <sawMedtronicStealthlink/sawMedtronicStealthlinkExport.h>

// forward declarations of Stealthlink types
namespace MNavStealthLink {
    class StealthServer;
    class Exam;
    class SurgicalPlan;
    template <typename T> class Subscription;
}


class CISST_EXPORT mtsMedtronicStealthlink: public mtsTaskFromSignal
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);


    friend class mtsStealthRegistration;
    friend class mtsStealthFrame;
    friend class mtsStealthTool;


    MNavStealthLink::StealthServer * StealthServerProxy;

    osaThread StealthServerProxyThread;

    void * StealthlinkRun(void *);

    // State data
    //mtsStealthTool ToolData;

    //mtsStealthFrame FrameData;

    //mtsStealthRegistration RegistrationData;

    mtsStealthProbeCal ProbeCal;

    class my_mtsDoubleVec:public myGenericObject, public mtsDoubleVec {
        public:
            my_mtsDoubleVec(size_type size_in) {this->SetSize(size_in);}
            virtual myGenericObject * operator=(const MNavStealthLink::DataItem & item_in);
    };

    // Other persistent data
    //my_mtsDoubleVec SurgicalPlan;  // entry point + target point

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
    //Tool * CurrentTool, * CurrentFrame;
    void GetTool(mtsStealthTool & tool) const;
    void GetFrame(mtsStealthFrame & frame) const;
    void GetProbeCalibration(mtsStealthProbeCal & probeCal) const {}


    // Class used to store registration data
    class Registration
    {
    public:
        mtsFrm3 Transformation;
        mtsBool Valid;
        mtsDouble PredictedAccuracy;
    };
    Registration RegistrationMember;

    // Class used to store exam info
    class ExamInformation: public myGenericObject
    {
    public:
        virtual myGenericObject * operator=(const MNavStealthLink::DataItem & item_in);
        mtsDouble3 VoxelScale;
        mtsInt3 Size;
        bool Valid;
    };
    //ExamInformation ExamInformationMember;

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

    // Stealthlink 2.0 callback

    class myCallback{
        private:
            mtsMedtronicStealthlink * my_parent;
        protected:
            template <typename T> friend class MNavStealthLink::Subscription;
            void operator()(const MNavStealthLink::DataItem& item_in);
        public:
            myCallback(mtsMedtronicStealthlink * parent_in):my_parent(parent_in) {}
    };

    myCallback myCallbackMember;

    //Stealthlink 2.0 Subscriptions

    MNavStealthLink::Subscription<MNavStealthLink::Instrument> * instrumentSubscription;
    MNavStealthLink::Subscription<MNavStealthLink::Frame> * frameSubscription;
    MNavStealthLink::Subscription<MNavStealthLink::Registration> * registrationSubscription;
    MNavStealthLink::Subscription<MNavStealthLink::Exam> * examSubscription;
    MNavStealthLink::Subscription<MNavStealthLink::SurgicalPlan> * surgicalPlanSubscription;


    struct less_DataItem : std::binary_function<const MNavStealthLink::DataItem *, const MNavStealthLink::DataItem *, bool>
    {
        bool operator() (const MNavStealthLink::DataItem * a, const MNavStealthLink::DataItem * b) const ;
     };


    typedef std::pair<const MNavStealthLink::DataItem * ,myGenericObject *> DataMapContainerItem;
    typedef std::map<const MNavStealthLink::DataItem * ,myGenericObject *,less_DataItem> DataMapContainer;
    typedef std::pair<DataMapContainer::iterator,bool> DataMapContainerInsertReturnValue;
    DataMapContainer myDataMap;

    typedef std::vector<DataMapContainer::iterator> myDataMapIteratorsContainer;
    myDataMapIteratorsContainer myToolIterators;
    myDataMapIteratorsContainer myFrameIterators;

    DataMapContainer::iterator myPlanIterator;

    struct less_myGenericObject : std::binary_function<const myGenericObject *, const myGenericObject *, bool>
    {
        bool operator() (const myGenericObject * a, const myGenericObject * b) const {
            if (typeid(a) == typeid(const mtsStealthTool *) && typeid(b) == typeid(const mtsStealthTool *)){
                return strcmp(dynamic_cast<const mtsStealthTool *>(a)->GetName(),dynamic_cast<const mtsStealthTool *>(b)->GetName()) == 0;
            }else if (typeid(a) == typeid(const mtsStealthFrame *) && typeid(b) == typeid(const mtsStealthFrame *)){
                return strcmp(dynamic_cast<const mtsStealthFrame *>(a)->GetName(),dynamic_cast<const mtsStealthFrame *>(b)->GetName()) == 0;
            }else{
                return typeid(a) == typeid(b);
            }
        }
     };

    typedef std::pair<const myGenericObject *, mtsStateTable *> StateTableMapContainerItem;
    typedef std::map<const myGenericObject *, mtsStateTable *,less_myGenericObject> StateTableMapContainer;
    typedef std::pair<StateTableMapContainer::iterator,bool> StateTableMapContainerInsertReturnValue;
    StateTableMapContainer myStateTableMap;

 public:
    mtsMedtronicStealthlink(const std::string & taskName);
    ~mtsMedtronicStealthlink();

    void Startup(void);

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
