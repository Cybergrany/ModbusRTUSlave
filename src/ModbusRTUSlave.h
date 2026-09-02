#ifndef ModbusRTUSlave_h
#define ModbusRTUSlave_h

#include "Arduino.h"
#include "ModbusADU.h"
#include "ModbusSlaveLogic.h"
#include "ModbusRTUComm.h"

class ModbusRTUSlave : public ModbusSlaveLogic {
  public:
    ModbusRTUSlave(Stream& serial, int dePin = -1, int rePin = -1);
    void setResponseDelay(unsigned long responseDelay);
    // Registers one optional classifier for application-specific frame shapes.
    // Set it once during setup; a single callback may recognize multiple
    // function codes and lengths. It is consulted only for queued backlog
    // recovery, not for normally gap-framed requests.
    void setAdditionalFrameCandidateFn(
        ModbusRTUFrameCandidateFn additionalFrameCandidate);
    void begin(uint8_t localUnitId, unsigned long baud, uint32_t config = SERIAL_8N1);
    bool poll();

  private:
    ModbusRTUComm _rtuComm;
    uint8_t _localUnitId = 0;
    unsigned long _responseDelay = 0;
    ModbusRTUFrameCandidateFn _additionalFrameCandidate = NULL;
    using ModbusSlaveLogic::processPdu;

};

#endif
