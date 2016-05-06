#include "Logging.h"
#include "stdio.h"

//------------------------------------------------------------------------
// Logging functions
void POG()
{
  printf("\n");
}

void POG(const char* str)
{
  puts(str);
}

void POG(long unsigned int val)
{
  printf("%lu", val);
}

void POG_LN()
{
  printf("\n");
}

void POG_LN(const char* str)
{
  printf("%s\n", str);
}

void POG_LN(long unsigned int val)
{
  printf("%lu\n", val);
}


