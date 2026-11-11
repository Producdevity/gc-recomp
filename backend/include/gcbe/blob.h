/* Translated-code image written by the AArch64 backend and mapped by the loader.
   Little-endian, all offsets from the start of the file. */
#ifndef GCBE_BLOB_H
#define GCBE_BLOB_H

#include <stdint.h>

#define GCBE_MAGIC 0x45424347u /* "GCBE" */
#define GCBE_VERSION 1u
/* Bumped whenever generated code changes; images from another translator are retranslated. */
#define GCBE_TRANSLATOR 28u

enum { GCBE_FLAG_VIRTUAL_TIME = 1u };

typedef struct gcbe_header {
    uint32_t magic, version, flags, translator;
    uint64_t dol_hash;          /* FNV-1a of the DOL, the runtime's gc_dol_hash */
    uint8_t dol_sha1[20];
    uint32_t function_count, import_count, reloc_count, code_size;
    uint32_t functions_offset, imports_offset, relocs_offset, code_offset;
    uint32_t file_size;
} gcbe_header;

/* code_offset of an entry is relative to the code section. */
typedef struct gcbe_function {
    uint32_t start, end, code_offset, code_size;
} gcbe_function;

/* Writes the 64-bit address of import `import` at code + offset. */
typedef struct gcbe_reloc {
    uint32_t offset, import;
} gcbe_reloc;

/* Imports: import_count NUL-terminated symbol names, concatenated. */

#endif
