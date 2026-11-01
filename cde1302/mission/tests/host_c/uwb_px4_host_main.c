/*
 * Laptop test driver for main/uwb_px4.c.  Reads commands from stdin, one per
 * line, and answers each on stdout (the pytest suite compares with Python):
 *
 *   A <hex datagram>              -> "1 <valid>" if parsed, "0" if refused
 *   P <ux> <uy>                   -> "<px> <py>" with the last alignment
 *   U <pose> <fix> <ex> <ey> <ez> <max>  -> "1" if usable, else "0"
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "uwb_px4.h"

int main(void)
{
    static char line[512];
    uwb_xform_t xf;
    memset(&xf, 0, sizeof xf);

    while (fgets(line, sizeof line, stdin)) {
        if (line[0] == 'A') {
            uint8_t buf[64];
            size_t n = 0;
            for (char *p = line + 2; p[0] && p[1] && p[0] != '\n' && n < sizeof buf; p += 2) {
                char hex[3] = {p[0], p[1], 0};
                buf[n++] = (uint8_t)strtoul(hex, NULL, 16);
            }
            if (uwb_align_parse(buf, n, &xf)) {
                printf("1 %d\n", xf.valid ? 1 : 0);
            } else {
                printf("0\n");
            }
        } else if (line[0] == 'P') {
            float ux, uy, px, py;
            sscanf(line + 1, "%f %f", &ux, &uy);
            uwb_xform_apply(&xf, ux, uy, &px, &py);
            printf("%.6f %.6f\n", (double)px, (double)py);
        } else if (line[0] == 'U') {
            int pose, fix;
            float ex, ey, ez, max_eop;
            sscanf(line + 1, "%d %d %f %f %f %f", &pose, &fix, &ex, &ey, &ez, &max_eop);
            nlink_reading_t r;
            memset(&r, 0, sizeof r);
            r.has_pose = pose != 0;
            r.has_fix = fix != 0;
            r.eop[0] = ex;
            r.eop[1] = ey;
            r.eop[2] = ez;
            printf("%d\n", uwb_reading_usable(&r, max_eop) ? 1 : 0);
        }
    }
    return 0;
}
