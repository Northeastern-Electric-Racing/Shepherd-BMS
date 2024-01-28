#include "eepromdirectory.h"
#include "m24c32.h"

void eepromInit()
{   
    eeprom_data[0].id =  (char*)("ROOT");     
    eeprom_data[0].size = 1;

    eeprom_data[1].id = (char*)("FAULTS");
    eeprom_data[1].size = 21;                      

    // Initialize EEPROM addresses given data and length

    int i = 1;
    int offset = 0;

    /* initialize root offset to zero */
    eeprom_data[0].address = EEPROM_ROOT_ADDR;

    /* continue through array, setting offsets *//* private funciton prototypes */
    while (eeprom_data[i].id != NULL)
    {
        offset += eeprom_data[i-1].size;
        eeprom_data[i].address = offset;
        i++;
    }

    // TODO ADD THIS BACK AND FIGURE OUT WHAT THE FUCK ITS TRYING TO DO
    //Initialize first byte of faults partition to contain the address of the end of the partition so that the 
    //log_fault function logs the first fault to the first 4 bytes in the partition.
    // eeprom_write_data_key((char*)("FAULTS"), (eeprom_data[eeprom_get_index((char*)("FAULTS"))]
    // + eeprom_data[eeprom_get_index((char*)("FAULTS"))].size - 3), 1);
}

uint16_t eeprom_get_index(char *key)
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

char* eeprom_get_key(int index)
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

bool eeprom_read_data_key(char *key, void *data, uint16_t size)
{
    if(!data) {
        return false;
    }
    /* read data from eeprom given key and size */
    int address = eeprom_get_index(key);
    eeprom_read(address, data, size);

    return true;

}

bool eeprom_read_data_address(uint16_t address, void *data, uint16_t size)
{
    if(!data) {
        return false;
    }
    /* read data from eeprom given index */
    eeprom_read(address, data, size);
    return true;
}

bool eeprom_write_data_key(char *key, void *data, uint16_t size)
{
    if(!data) {
        return false;
    }
    /* write data to eeprom given key, offset, and size of data */
    int address = eeprom_get_index(key);
    eeprom_write(address, data, size);
    return true;
}

bool eeprom_write_data_address(uint16_t address, void *data, uint16_t size)
{
    if(!data) {
        return false;
    }
    /* write data to eeprom given page, offset, and size of data */
    eeprom_write(address, data, size);
    return true;
}

void log_fault(uint32_t fault_code)
{
    uint32_t fault = fault_code;
    //The next address to write a fault to is located in the first byte of the FAULTS partition.
    uint8_t reg_to_write;
    eeprom_read_data_address(eeprom_get_index((char*)("FAULTS")), &reg_to_write, 1);

    uint8_t startIndex =  eeprom_data[eeprom_get_index((char*)("FAULTS"))].address;
    uint8_t size = eeprom_data[eeprom_get_index((char*)("FAULTS"))].size;

    /* if the index is at the end of the partition, wrap around (currently store 5 faults, so max = 5 + offset) */
    if (reg_to_write == size + startIndex - 3)
    {
        /* first byte of partition is the index of the most recent fault, faults begin at second byte */
        reg_to_write = startIndex + 1;
    }
    else
    {
        reg_to_write += 4;
    }

    /* write the fault code*/
    eeprom_write_data_address(reg_to_write, &fault, 4);
    /* update first byte of faults partition*/
    eeprom_write_data_address(startIndex, &reg_to_write, 1);
} 

void get_faults()
{
    uint8_t curr_reg;
    eeprom_read_data_key("FAULTS", &curr_reg, 1);

    uint8_t startAddress =  eeprom_data[eeprom_get_index((char*)("FAULTS"))].address;
    uint8_t size = eeprom_data[eeprom_get_index((char*)("FAULTS"))].size;

    /* read and store the faults */

    int currIter = 0;
    while (currIter < NUM_EEPROM_FAULTS)
    {
        eeprom_read_data_address(curr_reg, &eeprom_faults[currIter], 4);
        currIter++;

        /* if the index is at the end of the partition, wrap around (5 faults * 4 bytes per fault + offset - 3  for start of fault) */
        if (curr_reg == size + startAddress - 3)
        {                             
            /* first byte of partition is the index of the most recent fault, faults begin at second byte */
            curr_reg= startAddress + 1;
        }
        
        else
        {
            /* 4 bytes per fault */
            curr_reg += 4;
        }
    }
}

bool test_EEPROM(){
    //TODO: Write code for and verify this first, and then move onto fault log test after verification
    // Data read/write test
    // Check if data in memory address
        // If there is, store it so can re-write later
    // Write data to set address
    // Read data from set address
    // Compare written and read data and make sure they're identical
    // printf whether the written and read data values were identical
    // If there was previously data in the EEPROM chip, rewrite it

    // Define the root index of the EEPROM chip
    uint8_t reg_to_write;
    uint8_t root_address = eeprom_get_index((char*)("ROOT"));

    // Grab the initial data in the given address
    uint8_t initial_data = eeprom_read_data_address(root_address, &reg_to_write, 1);
    
    if (initial_data != 0){
        printf("Initial data read from EEPROM", initial_data);
    }

    // Write a known data value to a certain address in EEPROM and read it back
    uint8_t known_data = 22;
    eeprom_write_data_address(root_address, &reg_to_write, 1);
    uint8_t data_read = eeprom_read_data_address(root_address, &reg_to_write, 1);

    // Check whether the written/read data from the EEPROM matches
    if (data_read == known_data){
        printf("Data was successfully written and read from EEPROM");
    }
    else{
        printf("Data was not successfully written/read from EEPROM");
    }

    // Write data previously stored in EEPROM address back to EEPROM
    eeprom_write_data_address(root_address, &reg_to_write, 1);

    //TODO: Verify that simple read/write test code works before moving onto testing this
    // Fault read/write test
    // Check if there are faults currently stored
    // If there are, read the 5 faults and save them so can re-write later
    // Write a fault to the fault log and check that the fault was written
    // printf whether the fault was correctly written/read
    // If there was previously data in the fault log, rewrite it to the log
}