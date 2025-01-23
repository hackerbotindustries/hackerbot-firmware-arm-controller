#include "MyCobotLanguage.h"


MyCobotLanguage::MyCobotLanguage()
{

}

void MyCobotLanguage::clearLanguage()
{

}


int MyCobotLanguage::language()
{
    // begin
    //EEPROM.begin(EEPROM_SIZE);

    if (hasSeletecd()) {
        return language_val;
    } else {
        return selectLanguage();
    }
}


bool MyCobotLanguage::hasSeletecd()
{
    if (language_val != 0) {
        return true;
    } else { // = 0
        return false;
    }
}

int MyCobotLanguage::selectLanguage()
{
    delay(100);
    // begin to choose
    while (1) {
        // english
        {
            language_val = EN_NO;
            break;
        }
        // chinese
        {
            language_val = CN_NO;
            break;
        }
        delay(50);
    }

    //Serial.println(language_val);

    // save to eeprom
    setLanguage(language_val);

    delay(50);
    return language_val;

}

void MyCobotLanguage::setLanguage(int lan_val)
{

}