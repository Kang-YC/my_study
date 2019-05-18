#ifndef _NATURE_H_
#define _NATURE_H_


/*
This is the nature header of the TiEV autonomouse driving system
*/

#include "nature/unit.h"
#include "nature/timestamp.h"
#include "nature/angle.h"

namespace TiEV{

const int GRID_ROW = 401;
const int GRID_COL = 151;
const float GRID_RESOLUTION = 0.2;
const int CAR_CEN_COL = 75;
const int CAR_CEN_ROW = 300;
const float CAR_WHEEL_BASE = 2.3;
enum LCMCHANNLE
{
	DEFAULT = 0, //for NAVINFO CANINFO
	MAPPING, //for SICKMAP LUXMAP LASERMAP
	PLANNING, // for FUSIONMAP ESROBJINFO OBJECTLIST
	VISION, //for LANE TRAFFICLIGHT STOPLINE
	CONTROL //for REMOTECONTROL SLAMCONTROL ACC AIMPATH AIMPATHINT CANCONTROL
};

//
//
}
#endif // _NATURE_H_
