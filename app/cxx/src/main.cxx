/**
 * Simple C++ application to test C++ support.
 * This application should not be run (iostream not being redirected)
 * It only validate the linker scripts and recipes.
 */

#include <string>
#include <iostream>

extern "C" {

int
main(int argc, char **)
{
   // test for libc++
   std::string s("Hello World");
   std::cout << s << "\n";

   // test for libunwind
   try {
      if ( argc != 1 ) {
         throw 1;
      }
   }
   catch (int) {
      return -1;
   }

   return 0;
}

void __attribute__((noreturn))
exit(int)
{
   for(;;);
}

} // extern "C"
