#include <rthw.h>
#include <rtthread.h>
#include <stdint.h>

void vPortEnterCritical( void )
{
    rt_enter_critical();
}

void vPortExitCritical( void )
{
    rt_exit_critical();
}

void vPortRaiseBASEPRI( void )
{
    rt_hw_interrupt_disable();
}

uint32_t ulPortRaiseBASEPRI( void )
{
    return rt_hw_interrupt_disable();
}

void vPortSetBASEPRI( uint32_t ulNewMaskValue )
{
    rt_hw_interrupt_enable(ulNewMaskValue);
}
