#include "eepromdirectory.h"

void eepromInit()
{   
    // Initialize EEPROM addresses given data and length

    int i = 1;
    int offset = 0;

    /* initialize root offset to zero */
    eeprom_data[0].address = EEPROM_ROOT_ADDR;

    /* continue through array, setting offsets */
    while (eeprom_data[i].id != NULL)
    {
        eeprom_data[i].address = eeprom_data[i-1].address + eeprom_data[i-1].size;
        i++;
    }
}

uint16_t eepromGetAddress(char *key)
{
    /* find the index of the key in the eeprom_data array */
    int i = 0;
    while (eeprom_data[i].id != NULL)
    {
        if (eeprom_data[i].id == key)
        {
            return eeprom_data[i].address;
        }

        i++;
    }
    return -1;
}

uint16_t eepromGetIndex(char *key)
{
    /* find the index of the key in the eeprom_data array */
    int i = 0;
    while (eeprom_data[i].id != NULL)
    {
        if (eeprom_data[i].id == key)
        {
            return i;
        }

        i++;
    }
    return -1;
}

char* eepromGetKey(int index)
{
    /* find the key at the index in the eeprom_data array */
    int i = 0;
    while (eeprom_data[i].id != NULL)
    {
        if (eeprom_data[i].address == index)
        {
            return eeprom_data[i].id;
        }

        i++;
    }
    return NULL;
}

void eepromReadData(char *key, void *data, uint16_t size)
{
    if(!data) {
        return -1;
    }
    /* read data from eeprom given key and size */
    int address = eepromGetAddress(key);
    eeprom_read(address, data, size);

}

void eepromReadData(uint16_t address, void *data, uint16_t size)
{
    if(!data) {
        return -1;
    }
    /* read data from eeprom given index */
    eeprom_read(address, data, size);
    //EEPROM.get(index, data); // TODO will need update with new eeprom driver
}

void eepromWriteData(char *key, void *data, uint16_t size)
{
    if(!data) {
        return -1;
    }
    /* write data to eeprom given key, offset, and size of data */
    int address = eepromGetAddress(key);
    eeprom_write(address, data, size);
}

void eepromWriteData(uint16_t address, void *data, uint16_t size)
{
    if(!data) {
        return -1;
    }
    /* write data to eeprom given page, offset, and size of data */
    eeprom_write(address, data, size);
}

void logFault(uint32_t *fault_code)
{
    //Get starting address of faults, then add the number of entries currently in eeprom_faults to find next available address.
    uint16_t address = eepromGetAddress(const_cast<char*>("FAULTS")) + (sizeof(eeprom_faults) / 4);

    /* write the fault code*/
    eepromWriteData(address, fault_code, 4);
} 

void getFaults()
{
    //Get address of the faults partition
    uint16_t* address = eepromGetAddress(const_cast<char*>("FAULTS"));

    /* read and store the faults */
    int currIter = 0;
    uint16_t size = 4;
    while (currIter < NUM_EEPROM_FAULTS)
    {
        eepromReadData(*address, &eeprom_faults[currIter], size);
        currIter++;
        *address += size;
    }

}