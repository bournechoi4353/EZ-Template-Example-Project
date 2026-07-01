/*
 * Embeds path.jerryio path files (from the static/ folder) directly into the
 * program binary -- so no SD card is needed.  The bytes land in read-only data
 * and are exposed to C++ as
 *      <name>_txt_start  ...  <name>_txt_end
 * which the ASSET(<name>_txt) macro in include/jerryio_path.hpp turns into a
 * jerryio::asset you can pass to jerryio::follow_path().
 *
 * TO ADD A PATH:
 *   1. Put the exported file in  static/<name>.txt   (name = letters/underscores)
 *   2. Add a line below:         PATH_ASSET <name>
 *   3. Run `pros make clean` once (the build can't see that a .txt changed).
 *
 * If your file name has dots (path.jerryio exports as "path.jerryio.txt"), a
 * name with dots can't be a code symbol, so give a symbol-safe name AND the
 * real file name:
 *       PATH_ASSET path_jerryio, "path.jerryio.txt"
 * then use ASSET(path_jerryio_txt) in your code.
 */

/* PATH_ASSET <symbol_name> [, "<file_name_in_static>"]
 *   - one arg:  embeds static/<symbol_name>.txt
 *   - two args: embeds static/<file_name>, exposed under <symbol_name>_txt */
.macro PATH_ASSET name, file
    .section .rodata
    .balign 4
    .global \name\()_txt_start
    .global \name\()_txt_end
\name\()_txt_start:
    .ifb \file
    .incbin "static/\name\().txt"
    .else
    .incbin "static/\file"
    .endif
\name\()_txt_end:
    .byte 0                 /* guard byte after end (not counted in size) */
    .balign 4
.endm

/* ----- your paths ----- */
PATH_ASSET example_path
PATH_ASSET path_jerryio, "path.jerryio.txt"
PATH_ASSET pushbackv2
