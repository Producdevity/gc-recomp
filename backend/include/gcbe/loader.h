/* Maps code translated by the AArch64 backend (gcbe/blob.h) into the running process. */
#ifndef GCBE_LOADER_H
#define GCBE_LOADER_H

#include <stddef.h>
#include <stdint.h>

#include "gcrt/recomp.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct gcbe_program gcbe_program;

/* Loads and links a translated image. Runtime symbols are bound from the gcrt runtime;
   hle_ and wrap_ replacements are looked up in the executable (link it with exported
   symbols). Returns NULL with a message in `error` on failure. */
gcbe_program* gcbe_load(const char* path, char* error, size_t error_size);
unsigned gcbe_function_count(const gcbe_program*);
const gc_func_entry* gcbe_functions(const gcbe_program*);
uint64_t gcbe_dol_hash(const gcbe_program*);
uint32_t gcbe_flags(const gcbe_program*);
void gcbe_dol_sha1(const gcbe_program*, uint8_t out[20]);
/* True when `path` holds an image from this translator for the DOL with the given SHA-1
   (gcbe::sha1 of the DOL bytes) and the given flags; reads only the header. */
int gcbe_cache_valid(const char* path, const uint8_t dol_sha1[20], uint32_t flags);
/* Names the translated functions f_XXXXXXXX for perf (/tmp/perf-PID.map). */
void gcbe_write_perf_map(const gcbe_program*);

#ifdef __cplusplus
}
#endif

#endif
