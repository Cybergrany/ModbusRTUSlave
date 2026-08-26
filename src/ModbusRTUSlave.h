// Public Arduino-library include.
//
// The functional compatibility implementation deliberately remains at
// Comms/ModbusRTUSlave.h so the production OGM source can be carried here
// byte-for-byte. Existing sketches may continue to include
// <ModbusRTUSlave.h>; existing OGM code may continue to include
// "Comms/ModbusRTUSlave.h" during the later consumer migration.
#ifndef OGM_MODBUS_RTU_SLAVE_PUBLIC_H
#define OGM_MODBUS_RTU_SLAVE_PUBLIC_H

#include "Comms/ModbusRTUSlave.h"

#endif
