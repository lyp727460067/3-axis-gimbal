/**/
#include "main.h"

/**/
uint32_t Timer_GetCount(void) {
  return SysTick->VAL;
}

/**/
uint32_t Timer_CalDiff(uint32_t OldTime) {
  uint32_t c = SysTick->VAL;
  if (OldTime < c) {
    return (0x00ffffff - c) + OldTime;
  }
  return (OldTime - c);
}

/**/
uint32_t Timer_ToUs(uint32_t tx)
{
  return tx/72;
}

/**/
uint32_t Timer_FromUs(uint32_t us)
{
  return (us*72);
}

/**/
uint32_t Timer_ToMs(uint32_t tx)
{
  tx /= 72000;
  return tx;
}

/**/
uint32_t Timer_FromMs(uint32_t ms)
{
  return (ms *  72000);
}

/**/
static void _delay_100ms(void)
{
  int o_t = Timer_GetCount();
  int t_i;
  do
  {      t_i = Timer_CalDiff(o_t);}
  while(Timer_ToMs(t_i) < 100);
  
}
/**/
void Timer_Delay_ms(uint32_t x)
{
  while(x > 100)
  {
    _delay_100ms();
    x -= 100;
  };
  
  int o_t = Timer_GetCount();
  int t_i;
  do
  {      t_i = Timer_CalDiff(o_t);}
  while(Timer_ToMs(t_i) < x);
}

/**/
void Timer_Delay_us(uint32_t x)
{
  int o_t = Timer_GetCount();
  int t_i;
  x = x * 72;
  do
  {      t_i = Timer_CalDiff(o_t);}
  while((t_i) < x);
}