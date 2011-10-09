/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id$

  Author(s):  Peter Kazanzides, Anton Deguet
  Created on: 2011-07-14

  (C) Copyright 2011 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <cisstMultiTask/mtsTaskPeriodic.h>

#ifndef _mtsMedtronicStealthlinkExampleComponent_h
#define _mtsMedtronicStealthlinkExampleComponent_h

class mtsMedtronicStealthlinkExampleComponent: public mtsTaskPeriodic
{
public:

    struct ControllerStruct {
        mtsFunctionRead GetTool;
        mtsFunctionRead GetFrame;
    };

    struct ToolStruct {
        mtsFunctionRead GetMarkerCartesian;
        mtsFunctionRead GetPositionCartesian;
    };

    struct RegistrationStruct {
        mtsFunctionRead GetTransformation;
        mtsFunctionRead GetPredictedAccuracy;
        mtsFunctionRead GetValid;
    };

    struct ExamInformationStruct {
        mtsFunctionVoid RequestExamInformation;
        mtsFunctionRead GetVoxelScale;
        mtsFunctionRead GetSize;
        mtsFunctionRead GetValid;
    };

    mtsMedtronicStealthlinkExampleComponent(const std::string & name,
                                            double periodInSeconds);
    ~mtsMedtronicStealthlinkExampleComponent() {}

    void AddToolInterface(const std::string & toolName,
                          ToolStruct & functionSet);

    void Run(void);

    ControllerStruct Stealthlink;
    ToolStruct Pointer;
    ToolStruct Frame;
    RegistrationStruct Registration;
    ExamInformationStruct ExamInformation;
};

#endif // _mtsMedtronicStealthlinkExampleComponent_h
