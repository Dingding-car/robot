#ifndef __ROBOTBEHAVIOR_H__
#define __ROBOTBEHAVIOR_H__

#include "IBehavior.h"
// #include "RobotConfig.h"
#include "VoyCmd.h"

struct SensorData_t
{
    DOUBLE distances[ULTRASONICAMOUNT];
    BOOL enabled[ULTRASONICAMOUNT];
};

class RobotBehavior : public IBehavior {
private:
    CVoyCmd* m_cmd;
    bool m_showSensorData;
    SensorData_t m_ultraSonicData;

public:
    RobotBehavior(bool showSensorData = true) : m_cmd(nullptr), m_showSensorData(showSensorData) {}

    void SetShowSensor(bool enable);
    void SetCmd(CVoyCmd *pCmd) override;
    void AfterUpdateUSonic(DOUBLE *distances, BOOL *enabled, UINT state) override;
    void AfterUpdateInfrared(BOOL *data, BOOL *enabled, UINT state) override;
    void AfterUpdateMotorParam(UINT pos, UINT speed, UINT state) override;
    void AfterSendCommand(UCHAR *buffer, int length, UINT state) override;

    SensorData_t& GetSensorData();
};

#endif // __ROBOTBEHAVIOR_H__