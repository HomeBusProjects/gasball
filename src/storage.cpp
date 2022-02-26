#include "SparkFun_Qwiic_OpenLog_Arduino_Library.h"

#include <storage.h>

static OpenLog logger;

static bool is_present = false;

#define LOG_FILENAME "data.txt"

bool storage_begin() {
    if(logger.begin() && logger.getStatus() != 0xff) {
        Serial.print("[logger ");
        Serial.print(logger.getVersion());
        Serial.println("]");

        Serial.printf("SD status %0x\n", logger.getStatus());

        logger.append(String(LOG_FILENAME));

        is_present = true;
    } else {
        Serial.println("storage fail");
        is_present = false;
    }

    return is_present;
}

bool storage_writeln(const char *str) {
    if(is_present) {
        logger.println(str);
        return true;
    } else
        return false;
}

void storage_flush() {
    if(is_present)
        logger.syncFile();
}
