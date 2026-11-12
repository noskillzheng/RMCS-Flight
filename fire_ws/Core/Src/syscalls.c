/**
  ******************************************************************************
  * @file    syscalls.c
  * @brief   Minimal newlib syscall stubs for bare-metal STM32 builds
  ******************************************************************************
  */
#include <errno.h>
#include <sys/stat.h>

extern char end; /* Defined by linker script */

int _close(int file)
{
  (void)file;
  return -1;
}

int _fstat(int file, struct stat *st)
{
  (void)file;
  st->st_mode = S_IFCHR;
  return 0;
}

int _isatty(int file)
{
  (void)file;
  return 1;
}

int _lseek(int file, int ptr, int dir)
{
  (void)file;
  (void)ptr;
  (void)dir;
  return 0;
}

int _read(int file, char *ptr, int len)
{
  (void)file;
  (void)ptr;
  (void)len;
  return 0;
}

int _write(int file, char *ptr, int len)
{
  (void)file;
  (void)ptr;
  return len;
}

void *_sbrk(ptrdiff_t incr)
{
  static char *heap_end = &end;
  char *prev_heap_end = heap_end;
  register char *stack_ptr __asm("sp");

  if (heap_end + incr > stack_ptr)
  {
    errno = ENOMEM;
    return (void *)-1;
  }

  heap_end += incr;
  return (void *)prev_heap_end;
}
