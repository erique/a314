#pragma once
#ifndef KPRINTF_H
#define KPRINTF_H

#if DEBUG
#define ENABLE_KPRINTF
void kprintf(const char* fmt, ...);
void DumpBuffer(const uint8_t* buffer, uint32_t size);
#else // DEBUG
#define kprintf(...) do { } while(0)
#define DumpBuffer(...) do { } while(0)
#endif // DEBUG

#endif
