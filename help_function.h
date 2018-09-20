#ifndef MY_HELP_FUNCTION_H
#define MY_HELP_FUNCTION_H


#include <iostream>
#include <pcl/common/time.h>

//////////////////////////////////////////////////////////////////////////////////////////
#if defined(__linux__) || defined (TARGET_OS_MAC)

#include <unistd.h>
// Get the available memory size on Linux/BSD systems

size_t 
getTotalSystemMemory ()
{
  uint64_t memory = std::numeric_limits<size_t>::max ();

#ifdef _SC_AVPHYS_PAGES
  uint64_t pages = sysconf (_SC_AVPHYS_PAGES);
  uint64_t page_size = sysconf (_SC_PAGE_SIZE);
  
  memory = pages * page_size;
  
#elif defined(HAVE_SYSCTL) && defined(HW_PHYSMEM)
  // This works on *bsd and darwin.
  unsigned int physmem;
  size_t len = sizeof physmem;
  static int mib[2] = { CTL_HW, HW_PHYSMEM };

  if (sysctl (mib, ARRAY_SIZE (mib), &physmem, &len, NULL, 0) == 0 && len == sizeof (physmem))
  {
    memory = physmem;
  }
#endif

  if (memory > uint64_t (std::numeric_limits<size_t>::max ()))
  {
    memory = std::numeric_limits<size_t>::max ();
  }
  
  print_info ("Total available memory size: %lluMB.\n", memory / 1048576ull);
  return size_t (memory);
}
//////////////////////////////////////////////////////////////////////////////////////////
#define FPS_CALC(_WHAT_, buff) \
do \
{ \
    static unsigned count = 0;\
    static double last = pcl::getTime ();\
    double now = pcl::getTime (); \
    ++count; \
    if (now - last >= 1.0) \
    { \
      cerr << "Average framerate("<< _WHAT_ << "): " << double(count)/double(now - last) << " Hz. Queue size: " << buff.getSize () << "\n"; \
      count = 0; \
      last = now; \
    } \
}while(false)
//////////////////////////////////////////////////////////////////////////////////////////


#endif
