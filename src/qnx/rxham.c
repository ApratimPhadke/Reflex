/* rxham - "who guards the guardian?"
 *
 * Registers reflex-guardian with the QNX High Availability Manager. The HAM starts
 * the guardian, watches it, and restarts it if it ever dies. Because the plant
 * state and all counters live in shared memory, the restarted guardian resumes
 * the same world a few milliseconds later; the I/O node holds the actuator in
 * between.
 *
 *   ham                                      (start the HAM once, as root)
 *   rxham /full/path/reflex-guardian [args] (register + start the guardian)
 *   rxham -r                                 (unregister)
 *
 * QNX only. Link with -lham. SPDX-License-Identifier: MIT */
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ha/ham.h>

#define ENTITY "reflex-guardian"

int main(int argc, char **argv)
{
    if (argc < 2) { fprintf(stderr, "usage: rxham /full/path/reflex-guardian [args...]   |   rxham -r\n"); return 2; }

    if (ham_connect(0) == -1) { perror("rxham: ham_connect (is `ham` running?)"); return 1; }

    if (!strcmp(argv[1], "-r")) {
        ham_entity_t *e = ham_entity_handle(0 /* local node */, ENTITY, 0);
        if (!e || ham_detach(e, 0) == -1) perror("rxham: detach");
        else printf("rxham: %s is no longer monitored\n", ENTITY);
        ham_disconnect(0);
        return 0;
    }

    char line[1024] = "";
    for (int i = 1; i < argc; i++) {
        strncat(line, argv[i], sizeof line - strlen(line) - 2);
        if (i + 1 < argc) strcat(line, " ");
    }

    /* pid = -1: the HAM starts the process itself from `line` and monitors it */
    ham_entity_t *ent = ham_attach(ENTITY, 0, -1, line, 0);
    if (!ent) { perror("rxham: ham_attach"); ham_disconnect(0); return 1; }

    ham_condition_t *death = ham_condition(ent, CONDDEATH, "death", HREARMAFTERRESTART);
    if (!death) { perror("rxham: ham_condition"); ham_disconnect(0); return 1; }

    if (!ham_action_restart(death, "restart", line, HREARMAFTERRESTART)) {
        perror("rxham: ham_action_restart"); ham_disconnect(0); return 1;
    }

    printf("rxham: HAM now starts, monitors and restarts: %s\n", line);
    ham_disconnect(0);
    return 0;
}
