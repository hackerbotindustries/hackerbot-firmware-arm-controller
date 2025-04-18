#ifndef mycobotlanguage_h
#define mycobotlanguage_h
#define USE_M5_FONT_CREATOR

#include <Arduino.h>

#define Lan_Add     2
#define EEPROM_SIZE 64

#define EN_NO   1
#define CN_NO   2

class MyCobotLanguage
{

public:
    MyCobotLanguage();
    int language_val;

    void clearLanguage();
    int language();
    void setLanguage(int lan_num);

private:

    bool hasSeletecd();
    int selectLanguage();

};

#endif
