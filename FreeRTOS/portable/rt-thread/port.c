#include <FreeRTOS.h>

static rt_base_t level = 0;

void vPortEnterCritical( void )
{
    level = rt_hw_interrupt_disable();
}

void vPortExitCritical( void )
{
    rt_hw_interrupt_enable(level);
}

BaseType_t rt_err_to_freertos(rt_err_t rt_err)
{
    switch(-rt_err)
    {
        case RT_EOK:
            return pdPASS;
        case RT_ENOMEM:
            return errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY;
        default:
            return pdFAIL;
    }
}
