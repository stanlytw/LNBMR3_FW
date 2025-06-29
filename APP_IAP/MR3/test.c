#include <stdint.h>
#include <stdio.h>
#include "en_mr3_reg.h"

int main()
{
    uint8_t MR3[128] = {0};
    MR3[2] = 0x00;
    MR3[EN_MR3_REG_SC_02] = 0x01;
    printf("%d\n", MR3[2]);
    return 0;
}