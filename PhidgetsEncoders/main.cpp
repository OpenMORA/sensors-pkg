/* +---------------------------------------------------------------------------+
   |                 Open MORA (MObile Robot Arquitecture)                     |
   |                  University of Almeria ARM-eCar module                    |
   |                                                                           |
   |   Copyright (C) 2014  University of Almeria                               |
   +---------------------------------------------------------------------------+ */

#include "CPhidgetsEncoders.h"

int main(int argc ,char * argv[])
{
    const char * sMissionFile = "Mission.moos";
    const char * sMOOSName = "PhidgetsEncoders";
    switch(argc)
    {
    case 3:
        sMOOSName = argv[2];
    case 2:
        sMissionFile = argv[1];
    }

    CPhidgetsEncoders TheApp;
    TheApp.Run(sMOOSName,sMissionFile);
    return 0;
}
