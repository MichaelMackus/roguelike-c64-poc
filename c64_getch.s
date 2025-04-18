.export c64_getch
.export _c64_getch ; cc65 has different linking format

c64_getch:
_c64_getch:
    jsr $ff9f
    jsr $ffe4
    ldx #0
    rts
