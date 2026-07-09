/* embeds the path files from static/ into the binary so we don't need an
 * SD card. each PATH_ASSET line exposes <name>_txt_start/_end, which the
 * ASSET() macro in jerryio_path.hpp wraps into something you can follow.
 *
 * to add a path: drop static/<name>.txt, add PATH_ASSET <name> below, then
 * run `pros make clean` (the build can't tell a .txt changed).
 *
 * if the file name has dots, symbols can't have dots -- use the 2-arg form:
 *     PATH_ASSET path_jerryio, "path.jerryio.txt"
 * and use ASSET(path_jerryio_txt) in code.
 */

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
    .byte 0                 /* guard byte, not counted in size */
    .balign 4
.endm

/* ----- paths ----- */
PATH_ASSET example_path
PATH_ASSET path_jerryio, "path.jerryio.txt"
PATH_ASSET pushbackv2
