#include "EEPROM_MAP.h"
#include <stdio.h>

void func(int n, int addr);
 
int main(void)
{
    func(0, 0);
    func(1, 0);
    func(2, 0);
    func(3, 0);
    func(4, 0);
    func(5, 0);
    func(5, 2);
    func(5, 0x7e);
    func(6, 0x7e); /* page number out of range */
    func(5, 0xFF); /* addr out of range */

    int addr = 0xFF;
    if(IS_EN_ADDR(addr))
        func(0, addr);
    else
        printf("Value of Address Out of Range\n");

    addr = 0xFF + 1;
    if(IS_EN_ADDR(addr))
        func(0, addr);
    else
        printf("The value of Address is Out-of-Range\n");

    int n = 6;
    if(IS_EN_ADDR_PAGE(n))
        ;
    else
        printf("The value of Page number is Out-of-Range\n");

    return 0;
}

void func(int n, int addr)
{
    printf("page %d addr 0x%X at EEPROM 0x%X\n", n, addr, EN_ADDR_PAGE_IN_PAGE(n, addr));
}